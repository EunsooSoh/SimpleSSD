/*
 * FAST FTL 내부 상태 정합성 테스트
 *
 * 무작위 lane 쓰기, lane/페이지 순차 쓰기, trim, format을 섞어 실행하면서 매 단계마다
 * 아래 불변식을 검사한다.
 *  - 모든 (lpn, lane)의 최신 위치가 모델과 일치하고, 그 위치의 블록이 같은 lpn을 가진다
 *  - 블록들의 유효 sector 수 합 == 매핑된 sector 수
 *  - 사용 중인 블록은 데이터/SW/RW 로그 중 하나이다 (블록 누수 없음)
 *  - RW 로그 수가 FASTRWLogBlocks를 넘지 않는다
 *
 * 내부 멤버에 접근하기 위해 헤더를 포함하기 전에 private을 public으로 바꾼다.
 * PAL/DRAM 없이 상태만 검사하므로 bSendToPAL을 false로 두고 동작시킨다.
 *
 * 사용법: test_fast <SimpleSSD 설정 파일>   (예: config/sample.cfg)
 * 빌드:   cmake -DBUILD_TESTS=ON ... 후 make test_fast
 */

// FAST 내부 상태 정합성 검증용 테스트 (private 접근을 열어 두고 불변식을 검사한다)
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include "ftl/abstract_ftl.hh"
#include "ftl/common/block.hh"
#include "ftl/ftl.hh"
#include "pal/pal.hh"
#include "sim/config_reader.hh"
#define private public
#include "ftl/fast.hh"
#undef private

using namespace SimpleSSD;
using namespace SimpleSSD::FTL;

static int failures = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); failures++; if (failures > 20) exit(1);} } while (0)

static void verify(FAST &f, Parameter &p, std::set<uint64_t> &model, const char *when, int step) {
  uint32_t U = p.ioUnitInPage;
  CHECK(f.mappedSectors == model.size(), "[%s %d] mappedSectors %" PRIu64 " != model %zu", when, step, f.mappedSectors, model.size());
  uint64_t sum = 0;
  for (auto &e : f.mappedLanes) sum += e.second;
  CHECK(sum == model.size(), "[%s %d] mappedLanes sum %" PRIu64 " != %zu", when, step, sum, model.size());
  uint64_t total = p.totalLogicalBlocks * p.pagesInBlock;
  for (uint64_t lpn = 0; lpn < total; lpn++) {
    for (uint32_t lane = 0; lane < U; lane++) {
      uint32_t pbn, page;
      bool found = f.findLatest(lpn, lane, pbn, page);
      bool expect = model.count(lpn * U + lane) > 0;
      CHECK(found == expect, "[%s %d] lpn %" PRIu64 " lane %u found=%d model=%d", when, step, lpn, lane, found, expect);
      if (found) {
        Bitset bit(U);
        std::vector<uint64_t> lpns;
        bool v = f.blocks.at(pbn).getPageInfo(page, lpns, bit);
        CHECK(v && bit.test(lane) && lpns.at(lane) == lpn, "[%s %d] lpn %" PRIu64 " lane %u location invalid", when, step, lpn, lane);
      }
    }
  }
  uint64_t valid = 0;
  for (auto &b : f.blocks) valid += b.second.getValidPageCountRaw();
  CHECK(valid == model.size(), "[%s %d] valid sectors %" PRIu64 " != %zu", when, step, valid, model.size());
  // 블록 누수 검사: 사용 중인 블록은 데이터/SW/RW 로그 중 하나여야 한다
  size_t used = f.dataBlockMap.size() + (f.sw.valid ? 1 : 0) + f.rwLogs.size();
  CHECK(f.blocks.size() == used, "[%s %d] blocks %zu != referenced %zu (leak)", when, step, f.blocks.size(), used);
  CHECK(f.blocks.size() + f.nFreeBlocks == p.totalPhysicalBlocks, "[%s %d] block count mismatch", when, step);
  CHECK(f.nFreeBlocks == f.freeBlocks.size(), "[%s %d] nFreeBlocks mismatch", when, step);
  for (auto &e : f.rwMap) {
    CHECK(std::find(f.rwLogs.begin(), f.rwLogs.end(), e.second.first) != f.rwLogs.end(), "[%s %d] rwMap points outside rw logs", when, step);
  }
  CHECK(f.rwLogs.size() <= f.maxRWLogs, "[%s %d] too many rw logs", when, step);

  // victim 선택 검사: selectRWVictim()은 상태를 바꾸지 않으므로 매 단계 호출해 볼 수 있다.
  // 블록의 유효 sector 수는 Block이 아니라 rwMap에서 따로 세어 서로 대조한다.
  if (!f.rwLogs.empty()) {
    std::map<uint32_t, uint32_t> validByMap;
    for (auto &e : f.rwMap) validByMap[e.second.first]++;
    for (auto pbn : f.rwLogs) {
      CHECK(f.blocks.at(pbn).getValidPageCountRaw() == validByMap[pbn], "[%s %d] rw log %u valid count mismatch (block %u, rwMap %u)", when, step, pbn, f.blocks.at(pbn).getValidPageCountRaw(), validByMap[pbn]);
    }
    auto sel = f.selectRWVictim();
    CHECK(sel != f.rwLogs.end(), "[%s %d] selectRWVictim returned end", when, step);
    if (sel != f.rwLogs.end()) {
      if (f.rwGCPolicy == 0) {
        CHECK(sel == f.rwLogs.begin(), "[%s %d] FIFO did not pick the oldest rw log", when, step);
      } else {
        // greedy: 유효 sector 수가 최소이고, 최소가 여러 개면 가장 앞(오래된) 블록
        uint32_t minValid = UINT32_MAX;
        auto expect = f.rwLogs.begin();
        for (auto it = f.rwLogs.begin(); it != f.rwLogs.end(); ++it) {
          uint32_t v = validByMap[*it];
          if (v < minValid) { minValid = v; expect = it; }
        }
        CHECK(sel == expect, "[%s %d] greedy picked %u (valid %u), expected %u (valid %u)", when, step, *sel, validByMap[*sel], *expect, minValid);
      }
    }
  }
}

// 설정 파일을 읽어 FASTRWLogBlocks만 작은 값으로 바꾸고 FASTRWGCPolicy를 지정한
// 임시 설정 파일을 만든다 (RW 로그가 자주 가득 차서 merge 경로가 많이 실행되게 하기 위함)
static std::string makeTestConfig(const char *path, int policy) {
  std::ifstream in(path);
  std::string tmp = "test_fast.tmp.cfg";
  std::ofstream out(tmp);
  std::string line;
  bool replaced = false;
  bool policyReplaced = false;

  if (!in.is_open()) {
    printf("설정 파일을 열 수 없습니다: %s\n", path);
    exit(2);
  }

  while (std::getline(in, line)) {
    if (line.compare(0, 15, "FASTRWLogBlocks") == 0) {
      line = "FASTRWLogBlocks = 3";
      replaced = true;
    }
    else if (line.compare(0, 14, "FASTRWGCPolicy") == 0) {
      line = "FASTRWGCPolicy = " + std::to_string(policy);
      policyReplaced = true;
    }

    out << line << "\n";
  }

  if (!replaced) {
    printf("설정 파일에 FASTRWLogBlocks가 없습니다\n");
    exit(2);
  }

  if (!policyReplaced) {
    printf("설정 파일에 FASTRWGCPolicy가 없습니다\n");
    exit(2);
  }

  return tmp;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("사용법: %s <SimpleSSD 설정 파일>\n", argv[0]);
    return 2;
  }

  Parameter p;
  p.totalPhysicalBlocks = 60;
  p.totalLogicalBlocks = 30;
  p.pagesInBlock = 8;
  p.pageSize = 4096 * 4;
  p.ioUnitInPage = 4;
  p.pageCountToMaxPerf = 4;

  // RW 로그 victim 정책(0: FIFO, 1: greedy) 두 가지를 모두 검사한다
  for (int policy : {0, 1})
  for (int ioUnit : {4, 1}) {
    std::string cfg = makeTestConfig(argv[1], policy);
    ConfigReader conf;
    conf.init(cfg);
    std::remove(cfg.c_str());
    p.ioUnitInPage = ioUnit;
    FAST f(conf, p, nullptr, nullptr);
    f.bSendToPAL = false;
    std::set<uint64_t> model;
    std::mt19937_64 rng(1234 + ioUnit);
    uint64_t total = p.totalLogicalBlocks * p.pagesInBlock;
    Request req(p.ioUnitInPage);
    req.ioFlag.set();
    uint64_t tick = 0;
    int step = 0;
    auto addModel = [&](uint64_t lpn, Bitset &flag) {
      for (uint32_t l = 0; l < p.ioUnitInPage; l++) if (flag.test(l)) model.insert(lpn * p.ioUnitInPage + l);
    };
    for (int round = 0; round < 4000; round++) {
      int kind = rng() % 12;
      if (kind < 3) {
        // lane 단위 순차 쓰기: 페이지 순서로 lane을 하나씩 보낸다 (ICL이 보내는 패턴)
        uint64_t lbn = rng() % p.totalLogicalBlocks;
        uint32_t start = rng() % 3 == 0 ? rng() % p.pagesInBlock : 0;
        uint32_t len = 1 + rng() % (p.pagesInBlock - start);
        for (uint32_t o = start; o < start + len; o++) {
          for (uint32_t l = 0; l < p.ioUnitInPage; l++) {
            req.lpn = lbn * p.pagesInBlock + o;
            req.ioFlag.reset();
            req.ioFlag.set(l);
            f.writeInternal(req, tick);
            addModel(req.lpn, req.ioFlag);
            verify(f, p, model, "laneseq", step++);
          }
        }
      } else if (kind < 5) {
        // 페이지 전체 순차 쓰기
        uint64_t lbn = rng() % p.totalLogicalBlocks;
        for (uint32_t o = 0; o < p.pagesInBlock; o++) {
          req.lpn = lbn * p.pagesInBlock + o;
          req.ioFlag.set();
          f.writeInternal(req, tick);
          addModel(req.lpn, req.ioFlag);
          verify(f, p, model, "pageseq", step++);
        }
      } else if (kind < 10) {
        // 무작위 lpn, 무작위 lane 부분집합
        req.lpn = rng() % total;
        req.ioFlag.reset();
        while (req.ioFlag.none()) {
          for (uint32_t l = 0; l < p.ioUnitInPage; l++) if (rng() % 2) req.ioFlag.set(l);
        }
        f.writeInternal(req, tick);
        addModel(req.lpn, req.ioFlag);
        verify(f, p, model, "rand", step++);
      } else if (kind < 11) {
        req.lpn = rng() % total;
        f.trimInternal(req, tick);
        for (uint32_t l = 0; l < p.ioUnitInPage; l++) model.erase(req.lpn * p.ioUnitInPage + l);
        verify(f, p, model, "trim", step++);
      } else {
        LPNRange range;
        range.slpn = rng() % total;
        range.nlp = 1 + rng() % 20;
        if (range.slpn + range.nlp > total) range.nlp = total - range.slpn;
        f.format(range, tick);
        for (uint64_t lp = range.slpn; lp < range.slpn + range.nlp; lp++)
          for (uint32_t l = 0; l < p.ioUnitInPage; l++) model.erase(lp * p.ioUnitInPage + l);
        verify(f, p, model, "format", step++);
      }
      if (failures) { printf("stopped at round %d (policy %d ioUnit %d)\n", round, policy, ioUnit); return 1; }
    }
    printf("policy %d ioUnit %d OK: steps=%d switch=%" PRIu64 " partial=%" PRIu64 " full=%" PRIu64 " victims=%" PRIu64 "\n", policy, ioUnit, step, f.stat.switchMerge, f.stat.partialMerge, f.stat.fullMerge, f.stat.rwVictims);
  }
  return failures ? 1 : 0;
}
