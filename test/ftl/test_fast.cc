/*
 * FAST FTL 내부 상태 정합성 테스트
 *
 * 무작위 lane 쓰기, lane/페이지 순차 쓰기, trim, format을 섞어 실행하면서 매 단계마다
 * 아래 불변식을 검사한다.
 *  - 모든 (lpn, lane)의 최신 위치가 모델과 일치하고, 그 위치의 블록이 같은 lpn을 가진다
 *  - 블록들의 유효 sector 수 합 == 매핑된 sector 수
 *  - 사용 중인 블록은 데이터/SW/RW 로그 중 하나이다 (블록 누수 없음)
 *  - RW 로그 수가 FASTRWLogBlocks를 넘지 않는다
 *  - (쓰기 버퍼) 매핑된 sector는 NAND 최신 위치와 버퍼 중 정확히 한 곳에만 있다
 *  - (쓰기 버퍼) 버퍼는 lane 수보다 작고, 열린 RW 로그가 아닌 RW 로그는 모든 lane이 꽉 찼다
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
#include <iterator>
#include <list>
#include <limits>
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
      uint64_t key = lpn * U + lane;
      uint32_t pbn, page, physLane;
      bool found = f.findLatest(lpn, lane, pbn, page, physLane);
      bool buffered = f.inBuffer(key);
      bool expect = model.count(key) > 0;
      // NAND와 버퍼 중 정확히 한 곳에만 있어야 한다
      CHECK(!(found && buffered), "[%s %d] lpn %" PRIu64 " lane %u in both NAND and buffer", when, step, lpn, lane);
      CHECK((found || buffered) == expect, "[%s %d] lpn %" PRIu64 " lane %u found=%d buffered=%d model=%d", when, step, lpn, lane, found, buffered, expect);
      if (found) {
        Bitset bit(U);
        std::vector<uint64_t> lpns;
        bool v = f.blocks.at(pbn).getPageInfo(page, lpns, bit);
        // RW 로그 칸에는 sector 키가, 데이터 블록/SW 로그 칸에는 lpn이 있다
        bool isRW = f.rwMap.count(key) > 0;
        uint64_t stored = isRW ? key : lpn;
        CHECK(isRW || physLane == lane, "[%s %d] lpn %" PRIu64 " lane %u moved lane outside rw log", when, step, lpn, lane);
        CHECK(v && bit.test(physLane) && lpns.at(physLane) == stored, "[%s %d] lpn %" PRIu64 " lane %u location invalid", when, step, lpn, lane);
      }
    }
  }
  uint64_t valid = 0;
  for (auto &b : f.blocks) valid += b.second.getValidPageCountRaw();
  CHECK(valid + f.wbuf.size() == model.size(), "[%s %d] valid sectors %" PRIu64 " + buffer %zu != %zu", when, step, valid, f.wbuf.size(), model.size());
  CHECK(f.wbuf.size() == f.wbufIndex.size(), "[%s %d] buffer index size mismatch", when, step);
  for (size_t i = 0; i < f.wbuf.size(); i++) {
    CHECK(f.wbufIndex.at(f.wbuf[i]) == i, "[%s %d] buffer index mismatch", when, step);
  }
  if (f.useWriteBuffer) {
    CHECK(f.wbuf.size() < U, "[%s %d] buffer not flushed (%zu)", when, step, f.wbuf.size());
    // 4번: 열린 RW 로그가 아닌 RW 로그(= victim 후보)는 모든 lane이 꽉 차 있어야 한다
    for (auto pbn : f.rwLogs) {
      CHECK(f.isOpenRW(pbn) || f.isRWFull(pbn), "[%s %d] closed rw log %u is not full", when, step, pbn);
    }
    for (auto o : f.openRW) {
      CHECK(o < 0 || std::find(f.rwLogs.begin(), f.rwLogs.end(), (uint32_t)o) != f.rwLogs.end(), "[%s %d] open rw log outside rw logs", when, step);
    }
  } else {
    CHECK(f.wbuf.empty(), "[%s %d] buffer used while disabled", when, step);
  }
  // 블록 누수 검사: 사용 중인 블록은 데이터/SW/RW 로그 중 하나여야 한다
  size_t used = f.dataBlockMap.size() + (f.sw.valid ? 1 : 0) + f.rwLogs.size();
  CHECK(f.blocks.size() == used, "[%s %d] blocks %zu != referenced %zu (leak)", when, step, f.blocks.size(), used);
  CHECK(f.blocks.size() + f.nFreeBlocks == p.totalPhysicalBlocks, "[%s %d] block count mismatch", when, step);
  CHECK(f.nFreeBlocks == f.freeBlocks.size(), "[%s %d] nFreeBlocks mismatch", when, step);
  for (auto &e : f.rwMap) {
    CHECK(std::find(f.rwLogs.begin(), f.rwLogs.end(), e.second.pbn) != f.rwLogs.end(), "[%s %d] rwMap points outside rw logs", when, step);
  }
  CHECK(f.rwLogs.size() <= f.maxRWLogs, "[%s %d] too many rw logs", when, step);

  // victim 선택 검사: selectRWVictim()은 상태를 바꾸지 않으므로 매 단계 호출해 볼 수 있다.
  // 블록의 유효 sector 수는 Block이 아니라 rwMap에서 따로 세어 서로 대조한다.
  if (!f.rwLogs.empty()) {
    std::map<uint32_t, uint32_t> validByMap;
    for (auto &e : f.rwMap) validByMap[e.second.pbn]++;
    for (auto pbn : f.rwLogs) {
      CHECK(f.blocks.at(pbn).getValidPageCountRaw() == validByMap[pbn], "[%s %d] rw log %u valid count mismatch (block %u, rwMap %u)", when, step, pbn, f.blocks.at(pbn).getValidPageCountRaw(), validByMap[pbn]);
    }
    auto sel = f.selectRWVictim();
    // 열린 RW 로그는 후보가 아니므로, 후보가 하나도 없을 때만 end가 나올 수 있다
    bool anyClosed = false;
    for (auto pbn : f.rwLogs) if (!f.isOpenRW(pbn)) anyClosed = true;
    CHECK((sel != f.rwLogs.end()) == anyClosed, "[%s %d] selectRWVictim end mismatch", when, step);
    if (sel != f.rwLogs.end()) {
      CHECK(!f.isOpenRW(*sel), "[%s %d] victim is an open rw log", when, step);
      if (f.rwGCPolicy == 0) {
        auto first = f.rwLogs.begin();
        while (f.isOpenRW(*first)) ++first;
        CHECK(sel == first, "[%s %d] FIFO did not pick the oldest closed rw log", when, step);
      } else if (f.rwGCPolicy == 1) {
        // greedy: 유효 sector 수가 최소이고, 최소가 여러 개면 가장 앞(오래된) 블록
        uint32_t minValid = UINT32_MAX;
        auto expect = f.rwLogs.begin();
        for (auto it = f.rwLogs.begin(); it != f.rwLogs.end(); ++it) {
          if (f.isOpenRW(*it)) continue;
          uint32_t v = validByMap[*it];
          if (v < minValid) { minValid = v; expect = it; }
        }
        CHECK(sel == expect, "[%s %d] greedy picked %u (valid %u), expected %u (valid %u)", when, step, *sel, validByMap[*sel], *expect, minValid);
      } else {
        // Independent oracle: count NAND versions from logical mappings minus
        // DRAM-only versions, rather than calling production findLatest/cost.
        std::map<uint64_t, uint64_t> nandByLbn;
        for (auto &e : f.mappedLanes) nandByLbn[e.first / p.pagesInBlock] += e.second;
        for (auto key : f.wbuf) nandByLbn[key / U / p.pagesInBlock]--;
        auto expect = f.rwLogs.end();
        long double best = std::numeric_limits<long double>::infinity();
        for (auto it = f.rwLogs.begin(); it != f.rwLogs.end(); ++it) {
          if (f.isOpenRW(*it)) continue;
          std::set<uint64_t> lbns;
          bool moved = false;
          uint64_t valid = 0, written = 0, copies = 0;
          for (auto &e : f.rwMap) if (e.second.pbn == *it) {
            lbns.insert(e.first / U / p.pagesInBlock);
            moved |= e.second.relocated;
            valid++;
          }
          for (uint32_t lane = 0; lane < U; lane++)
            written += f.blocks.at(*it).getNextWritePageIndex(lane);
          for (auto lbn : lbns) copies += nandByLbn[lbn];
          long double score = written ? (long double)copies / written :
                              std::numeric_limits<long double>::infinity();
          if (f.useRWCompaction && !moved && valid > 0 && written > valid)
            score = std::min(score, (long double)valid / (written - valid));
          if (expect == f.rwLogs.end() || score < best - 1e-12L) {
            expect = it; best = score;
          }
        }
        CHECK(sel == expect, "[%s %d] cost selection differs from independent oracle", when, step);
      }
    }
  }
}

// 설정 파일을 읽어 FASTRWLogBlocks만 작은 값으로 바꾸고 FASTRWGCPolicy를 지정한
// 임시 설정 파일을 만든다 (RW 로그가 자주 가득 차서 merge 경로가 많이 실행되게 하기 위함)
static std::string makeTestConfig(const char *path, int policy, int buffer, int compaction, int common = 0) {
  std::ifstream in(path);
  std::string tmp = "test_fast.tmp.cfg";
  std::ofstream out(tmp);
  std::string line;
  bool replaced = false;
  bool policyReplaced = false;
  bool bufferReplaced = false;

  if (!in.is_open()) {
    printf("설정 파일을 열 수 없습니다: %s\n", path);
    exit(2);
  }

  while (std::getline(in, line)) {
    // 새 옵션이 없는 이전 설정 파일도 테스트할 수 있도록 버퍼 옵션 뒤에 삽입한다.
    if (line.compare(0, 16, "FASTRWCompaction") == 0) continue;
    if (line.compare(0, 16, "FASTCommonWarmup") == 0) continue;
    if (common) {
      if (line.compare(0, 9, "FillRatio") == 0) line = "FillRatio = 0.9";
      if (line.compare(0, 16, "InvalidPageRatio") == 0) line = "InvalidPageRatio = 0.3";
      if (line.compare(0, 11, "FillingMode") == 0) line = "FillingMode = 1";
    }
    if (line.compare(0, 15, "FASTRWLogBlocks") == 0) {
      line = "FASTRWLogBlocks = 3";
      replaced = true;
    }
    else if (line.compare(0, 14, "FASTRWGCPolicy") == 0) {
      line = "FASTRWGCPolicy = " + std::to_string(policy);
      policyReplaced = true;
    }
    else if (line.compare(0, 15, "FASTWriteBuffer") == 0) {
      line = "FASTWriteBuffer = " + std::to_string(buffer) +
             "\nFASTRWCompaction = " + std::to_string(compaction) +
             "\nFASTCommonWarmup = " + std::to_string(common);
      bufferReplaced = true;
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

  if (!bufferReplaced) {
    printf("설정 파일에 FASTWriteBuffer가 없습니다\n");
    exit(2);
  }

  return tmp;
}

// 작은 실제 쓰기 이력으로 압축/전체 병합 선택과 매핑 보존을 따로 검증한다.
static void testRWCompaction(const char *path) {
  for (int buffer : {0, 1})
  for (int lanes : {1, 4})
  for (int scenario = 0; scenario < 6; scenario++) {
    if (scenario == 4 && (buffer || lanes == 1)) continue;
    std::string cfg = makeTestConfig(path, 0, buffer, 1);
    ConfigReader conf;
    conf.init(cfg);
    std::remove(cfg.c_str());
    Parameter p;
    p.totalPhysicalBlocks = 24;
    p.totalLogicalBlocks = 8;
    p.pagesInBlock = 8;
    p.ioUnitInPage = lanes;
    p.pageSize = 4096 * lanes;
    p.pageCountToMaxPerf = 4;
    FAST f(conf, p, nullptr, nullptr);
    f.bSendToPAL = false;
    // 하나의 로그를 가득 채워서 GC의 교체 경로를 명확하게 만든다.
    f.maxRWLogs = 1;
    f.nOpenSlots = 1;
    f.openRW.assign(1, -1);
    std::set<uint64_t> model;
    Request req(lanes);
    uint64_t tick = 0;
    for (uint32_t page = 0; page < p.pagesInBlock; page++) {
      req.lpn = page;
      req.ioFlag.set();
      f.writeInternal(req, tick);
      for (int lane = 0; lane < lanes; lane++) model.insert(page * lanes + lane);
    }
    const uint32_t oldData = f.dataBlockMap.at(0);
    // 0: 높은 무효 비율, 1: 모두 무효, 2: 복사량/회수량 비율 불리,
    // 3: 무효 데이터 없음, 4: 요청 lane은 모두 유효하여 복사해도 공간이 안 남음,
    // 5: 압축 이후 호스트가 다시 쓰면 재이동 제한이 새 버전에 남지 않음.
    // 버퍼 모드 시나리오 0은 논리 lane 0을 여러 물리 lane에 배치한다.
    for (uint32_t page = 0; page < p.pagesInBlock; page++) {
      for (int lane = 0; lane < lanes; lane++) {
        uint64_t lpn = 1;
        uint32_t logicalLane = lane;
        if ((scenario == 0 || scenario == 5) && buffer) {
          lpn = 1 + lane;
          logicalLane = 0;
        } else if (scenario == 2) {
          lpn = page % 7;
        } else if (scenario == 3) {
          lpn = page;
        } else if (scenario == 4 && lane == 0) {
          lpn = page;
        }
        f.writeRW(lpn, logicalLane);
      }
    }
    CHECK(f.rwLogs.size() == 1, "focused: expected exactly one log");
    uint32_t victim = f.rwLogs.front();
    CHECK(f.isRWFull(victim), "focused: victim should be full");
    if (scenario == 1) {
      req.lpn = 1;
      f.trimInternal(req, tick);
      for (int lane = 0; lane < lanes; lane++) model.erase(lanes + lane);
    }
    // ensureOpenRW가 하는 닫기와 반환 블록 연결을 여기서 직접 수행한다.
    f.openRW[0] = -1;
    size_t valid = f.rwMap.size();
    uint32_t freeBefore = f.nFreeBlocks;
    auto result = f.mergeRWVictim(buffer ? UINT32_MAX : 0);
    if (scenario == 0 || scenario == 5) {
      CHECK(result >= 0, "focused: sparse victim should compact");
      if (result >= 0 && buffer) f.openRW[0] = result;
      CHECK(f.stat.rwCompactions == 1, "focused: missing compaction count");
      CHECK(f.stat.rwCompactionCopies == valid, "focused: copied count does not match latest sectors");
      CHECK(f.stat.fullMerge == 0, "focused: compaction unexpectedly merged data block");
      CHECK(f.dataBlockMap.at(0) == oldData, "focused: compaction replaced original data block");
      CHECK(f.nFreeBlocks == freeBefore, "focused: replacement leaked a spare block");
      CHECK(f.rwLogs.size() == 1, "focused: compaction exceeded log budget");
      for (auto &entry : f.rwMap) {
        CHECK(entry.second.pbn == (uint32_t)result, "focused: stale source mapping after compaction");
        CHECK(entry.second.relocated, "focused: relocated version not marked");
        if (!buffer) CHECK(entry.second.lane == entry.first % lanes, "focused: nonbuffer changed logical lane");
      }
      if (buffer && lanes > 1) {
        bool differentLane = false;
        for (auto &entry : f.rwMap) differentLane |= entry.second.lane != entry.first % lanes;
        CHECK(differentLane, "focused: cross-lane relocation path was not exercised");
      }
      verify(f, p, model, "compact", lanes + buffer);

      if (scenario == 5) {
        CHECK(f.blocks.count(victim) == 0, "focused: compacted source still referenced");
        req.lpn = 1;
        req.ioFlag.reset();
        req.ioFlag.set(0);
        f.writeInternal(req, tick);
        // 버퍼 ON에서도 새 버전이 NAND에 기록된 후의 플래그를 검사한다.
        if (buffer) f.flushWriteBuffer();
        auto latest = f.rwMap.find((uint64_t)lanes);
        CHECK(latest != f.rwMap.end(), "focused: host overwrite did not return to RW log");
        if (latest != f.rwMap.end()) {
          CHECK(!latest->second.relocated, "focused: new host version retained relocation guard");
        }
        verify(f, p, model, "compact-host-overwrite", lanes + buffer);
        continue;
      }

      // 살아남은 기존 버전을 그대로 둔 채 다시 채운다. 두 번째 이동 대신
      // full merge로 돌아가서 찬 데이터가 끝없이 재이동하지 않아야 한다.
      for (uint32_t page = 1; page < p.pagesInBlock; page++) {
        for (int lane = 0; lane < lanes; lane++) f.writeRW(buffer ? 5 : 2, lane);
      }
      f.openRW[0] = -1;
      CHECK(f.isRWFull(f.rwLogs.front()), "focused: repeated-relocation victim not full");
      victim = f.rwLogs.front();
      result = f.mergeRWVictim(buffer ? UINT32_MAX : 0);
      CHECK(result < 0, "focused: same cold version was compacted twice");
      CHECK(f.stat.rwCompactions == 1 && f.stat.fullMerge == 1,
            "focused: repeated-relocation guard did not full merge");
    } else {
      CHECK(result < 0, "focused: unsuitable victim should not compact (scenario %d)", scenario);
      CHECK(f.stat.rwCompactions == 0, "focused: unsuitable victim counted as compacted");
      if (scenario == 1) {
        CHECK(f.stat.fullMerge == 0, "focused: empty victim required unnecessary full merge");
        CHECK(f.nFreeBlocks == freeBefore + 1, "focused: empty victim was not reclaimed");
      } else {
        CHECK(f.stat.fullMerge == 1, "focused: expected full-merge fallback (scenario %d)", scenario);
      }
    }
    CHECK(f.blocks.count(victim) == 0, "focused: erased victim still referenced");
    verify(f, p, model, "compact-fallback", scenario);
  }
  printf("RW compaction focused cases OK\n");
}

static void testCostVictim(const char *path) {
  CHECK(FAST::lowerRWCost(1, 3, 1, 2), "rational order incorrect");
  CHECK(!FAST::lowerRWCost(2, 4, 1, 2), "rational tie incorrect");
  CHECK(FAST::lowerRWCost(UINT64_MAX - 1, UINT64_MAX, UINT64_MAX, UINT64_MAX - 1),
        "rational comparison overflowed");
  CHECK(!FAST::lowerRWCost(0, 0, 1, 2) && FAST::lowerRWCost(1, 2, 0, 0),
        "zero-space candidate ordering incorrect");
  for (int buffer : {0, 1})
  for (int compact : {0, 1})
  for (int sparse : {0, 1}) {
    std::string cfg = makeTestConfig(path, 2, buffer, compact);
    ConfigReader conf;
    conf.init(cfg); std::remove(cfg.c_str());
    Parameter p;
    p.totalPhysicalBlocks = 60; p.totalLogicalBlocks = 30;
    p.pagesInBlock = 8; p.ioUnitInPage = 1;
    p.pageSize = 4096; p.pageCountToMaxPerf = 4;
    FAST f(conf, p, nullptr, nullptr);
    f.bSendToPAL = false;
    std::set<uint64_t> model;
    Request req(1); req.ioFlag.set();
    uint64_t tick = 0;
    for (uint64_t lpn = 0; lpn < 16; lpn++) {
      if (sparse && lpn >= 8 && (lpn < 9 || lpn > 12)) continue;
      req.lpn = lpn; f.writeInternal(req, tick); model.insert(lpn);
    }
    // A: 3 live sectors, full LBN cost 8. B: 4 live sectors,
    // full LBN cost 4 (sparse) or 8 (dense). Both logs used 8 slots.
    for (int i = 0; i < 8; i++) f.writeToRW(1 + i % 3, 0);
    for (int i = 0; i < 8; i++) f.writeToRW(9 + i % 4, 0);
    auto a = f.rwLogs.begin(); auto b = std::next(a);
    f.rwGCPolicy = 1;
    CHECK(f.selectRWVictim() == a, "fixture did not distinguish greedy");
    f.rwGCPolicy = 2;
    uint64_t before = f.warmupStateHash();
    auto selected = f.selectRWVictim(0);
    CHECK(selected == (sparse ? b : a), "cost policy chose wrong victim");
    CHECK(f.warmupStateHash() == before, "candidate evaluation changed state");
    auto costA = f.estimateRWCost(*a, 0), costB = f.estimateRWCost(*b, 0);
    CHECK(costA.mergeCopies == 8 && costA.validSectors == 3 && costA.fullSpace == 8,
          "A full merge estimate incorrect");
    CHECK(costB.mergeCopies == (uint64_t)(sparse ? 4 : 8) && costB.validSectors == 4,
          "B full merge estimate incorrect");
    CHECK(!costB.compact, "equal/worse relocation should prefer full merge");
    int64_t replacement = f.mergeRWVictim(0);
    if (buffer && replacement >= 0) f.openRW[0] = replacement;
    CHECK(f.stat.rwCostSelections == 1 && f.stat.rwCostDifferentFromGreedy == (uint64_t)sparse,
          "cost-vs-greedy selection statistics incorrect");
    verify(f, p, model, "cost-victim", sparse + compact + buffer);
  }
  printf("Cost victim: 8 focused cases and overflow/tie checks OK\n");
}

static void testCommonWarmup(const char *path) {
  for (int buffer : {0, 1})
  for (int policy : {0, 1, 2})
  for (int lanes : {1, 4}) {
    ConfigReader configs[2];
    for (int c : {0, 1}) {
      std::string cfg = makeTestConfig(path, policy, buffer, c, 1);
      configs[c].init(cfg);
      std::remove(cfg.c_str());
    }
    Parameter p;
    p.totalPhysicalBlocks = 60; p.totalLogicalBlocks = 30;
    p.pagesInBlock = 8; p.ioUnitInPage = lanes;
    p.pageSize = 4096 * lanes; p.pageCountToMaxPerf = 4;
    FAST baseline(configs[0], p, nullptr, nullptr);
    FAST moving(configs[1], p, nullptr, nullptr);
    baseline.initialize(); moving.initialize();
    CHECK(baseline.warmupStateHash() == moving.warmupStateHash(),
          "common warmup states differ: buffer %d policy %d lanes %d", buffer, policy, lanes);
    CHECK(!baseline.useRWCompaction && moving.useRWCompaction,
          "common warmup did not restore runtime compaction");
    CHECK(baseline.rwGCPolicy == (uint32_t)policy && moving.rwGCPolicy == (uint32_t)policy,
          "common warmup did not restore runtime victim policy");
    std::set<uint64_t> model;
    // Match the config reader's float arithmetic in FAST::initialize().
    uint64_t filled = p.totalLogicalBlocks * p.pagesInBlock *
                     configs[0].readFloat(CONFIG_FTL, FTL_FILL_RATIO);
    for (uint64_t lpn = 0; lpn < filled; lpn++)
      for (int lane = 0; lane < lanes; lane++) model.insert(lpn * lanes + lane);
    verify(baseline, p, model, "common-baseline", lanes);
    verify(moving, p, model, "common-moving", lanes);
    for (FAST *f : {&baseline, &moving}) {
      CHECK(f->stat.fullMerge == 0 && f->stat.rwCompactions == 0 &&
            f->stat.hostWritePages == 0, "warmup counters leaked into measurements");
      uint64_t erases = 0;
      for (auto &entry : f->blocks) erases += entry.second.getEraseCount();
      for (auto &block : f->freeBlocks) erases += block.getEraseCount();
      CHECK(erases > 0, "common warmup did not exercise GC");
      for (auto &entry : f->rwMap)
        CHECK(!entry.second.relocated, "compaction ran during common warmup");
    }
    if (!moving.rwMap.empty()) {
      uint64_t before = moving.warmupStateHash();
      auto &loc = moving.rwMap.begin()->second;
      loc.relocated = true;
      CHECK(before != moving.warmupStateHash(), "warmup fingerprint missed relocation state");
      loc.relocated = false;
    }
  }
  printf("Common Greedy warmup: 12 paired cases OK\n");
}

static void testGCWait(const char *path) {
  ConfigReader conf;
  conf.init(path);
  Parameter p;
  p.totalPhysicalBlocks = 60; p.totalLogicalBlocks = 30;
  p.pagesInBlock = 8; p.ioUnitInPage = 4;
  p.pageSize = 4096 * 4; p.pageCountToMaxPerf = 4;
  FAST f(conf, p, nullptr, nullptr);
  f.bSendToPAL = false;
  f.batch.doneTick = 150;
  f.recordGCWait(100, 0);
  CHECK(f.stat.gcEvents[0] == 0, "warmup leaked into GC wait counters");
  f.bSendToPAL = true;
  f.recordGCWait(100, 0);
  f.recordGCWait(120, 0);
  f.recordGCWait(90, 1);
  f.recordGCWait(150, 2);
  CHECK(f.stat.gcEvents[0] == 2 && f.stat.gcWaitPs[0] == 80 &&
        f.stat.gcMaxWaitPs[0] == 50, "SW GC wait accumulation incorrect");
  CHECK(f.stat.gcEvents[1] == 1 && f.stat.gcWaitPs[1] == 60 &&
        f.stat.gcEvents[2] == 1 && f.stat.gcWaitPs[2] == 0,
        "GC wait paths/zero-duration events incorrect");
  std::vector<Stats> names;
  std::vector<double> values;
  f.getStatList(names, "ftl."); f.getStatValues(values);
  CHECK(names.size() == values.size(), "GC stat names/values misaligned");
  for (size_t i = 0; i < names.size(); i++) {
    if (names[i].name == "ftl.fast.gc.all.wait_ps")
      CHECK(values[i] == 140, "total GC wait double counted or omitted a path");
    if (names[i].name == "ftl.fast.gc.all.events")
      CHECK(values[i] == 4, "total GC event count incorrect");
  }
  f.resetStatValues();
  CHECK(f.stat.gcEvents[0] == 0 && f.stat.gcWaitPs[1] == 0 &&
        f.stat.gcMaxWaitPs[0] == 0, "GC wait reset failed");
  printf("GC wait: warmup exclusion, path totals, maximum and reset OK\n");
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("사용법: %s <SimpleSSD 설정 파일>\n", argv[0]);
    return 2;
  }

  testGCWait(argv[1]);
  testCommonWarmup(argv[1]);
  testCostVictim(argv[1]);
  testRWCompaction(argv[1]);
  if (failures) return 1;

  Parameter p;
  p.totalPhysicalBlocks = 60;
  p.totalLogicalBlocks = 30;
  p.pagesInBlock = 8;
  p.pageSize = 4096 * 4;
  p.ioUnitInPage = 4;
  p.pageCountToMaxPerf = 4;

  // 기존 FIFO/greedy와 버퍼 경로를 압축 OFF/ON 모두에서 검사한다.
  for (int compaction : {0, 1})
  for (int buffer : {0, 1})
  for (int policy : {0, 1, 2})
  for (int ioUnit : {4, 1}) {
    std::string cfg = makeTestConfig(argv[1], policy, buffer, compaction);
    ConfigReader conf;
    conf.init(cfg);
    std::remove(cfg.c_str());
    p.ioUnitInPage = ioUnit;
    FAST f(conf, p, nullptr, nullptr);
    CHECK(f.useRWCompaction == (compaction != 0), "compaction config was not applied");
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
    for (int round = 0; round < (policy == 2 ? 500 : 4000); round++) {
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
      if (failures) { printf("stopped at round %d (compaction %d buffer %d policy %d ioUnit %d)\n", round, compaction, buffer, policy, ioUnit); return 1; }
    }
    printf("compaction %d buffer %d policy %d ioUnit %d OK: steps=%d switch=%" PRIu64 " partial=%" PRIu64 " full=%" PRIu64 " victims=%" PRIu64 " compactions=%" PRIu64 " compactCopies=%" PRIu64 " fallbacks=%" PRIu64 " rwSectors=%zu buffered=%zu\n", compaction, buffer, policy, ioUnit, step, f.stat.switchMerge, f.stat.partialMerge, f.stat.fullMerge, f.stat.rwVictims, f.stat.rwCompactions, f.stat.rwCompactionCopies, f.stat.rwCompactionFallbacks, f.rwMap.size(), f.wbuf.size());
  }
  return failures ? 1 : 0;
}
