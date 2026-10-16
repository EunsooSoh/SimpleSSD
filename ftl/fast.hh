/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __FTL_FAST__
#define __FTL_FAST__

#include <cinttypes>
#include <list>
#include <random>
#include <unordered_map>
#include <vector>

#include "ftl/abstract_ftl.hh"
#include "ftl/common/block.hh"
#include "ftl/ftl.hh"
#include "pal/pal.hh"

namespace SimpleSSD {

namespace FTL {

// 초기화 시 랜덤 채우기에 쓰는 난수 시드 (매 실행 같은 결과를 얻기 위해 고정)
static const uint64_t FAST_RANDOM_SEED = 20260218;

/*
 * FAST (Fully-Associative Sector Translation) FTL의 멀티채널 구현.
 *
 * PAL의 SuperblockSize 설정에 의해 FTL이 보는 블록 1개는 채널별 물리 블록을
 * 묶은 슈퍼블록이다. 슈퍼블록의 한 페이지는 lane(= ioUnit, 채널 1개 분량)
 * 여러 개로 이루어지고, lane마다 NAND 블록이 따로 있으므로 lane별로 독립된
 * 순차 쓰기 포인터를 가질 수 있다.
 *
 * 논문의 "sector"는 (lpn, lane) 한 쌍에 대응한다. 모든 채널이 하나의 로그
 * 구조(SW 로그 1개 + RW 로그 N개)를 공유하며, 각 로그 블록은 슈퍼블록이다.
 */
class FAST : public AbstractFTL {
 private:
  PAL::PAL *pPAL;

  ConfigReader &conf;

  // 블록 레벨 매핑 테이블 (논리 블록 번호 -> 데이터 블록 번호)
  std::unordered_map<uint64_t, uint32_t> dataBlockMap;

  // 사용 중인 블록과 비어 있는 블록
  std::unordered_map<uint32_t, Block> blocks;
  std::list<Block> freeBlocks;
  uint32_t nFreeBlocks;

  // SW(순차 쓰기) 로그 블록: sector의 페이지 위치가 논리 offset과 항상 같다.
  // 순차 여부는 lane마다 따로 판단하므로 다음 offset도 lane별로 가진다.
  struct {
    bool valid;
    uint32_t pbn;
    uint64_t lbn;
  } sw;
  std::vector<uint32_t> swNext;

  // RW(랜덤 쓰기) 로그 블록: 가장 앞이 가장 오래된 블록(victim 후보)
  std::list<uint32_t> rwLogs;
  uint32_t maxRWLogs;

  // RW 로그의 섹터 매핑 테이블 (lpn * lane 수 + lane -> (블록, 페이지))
  std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> rwMap;

  // 새 블록을 할당할 때 chip(= blockIdx % pageCountToMaxPerf)을 돌아가며 고른다
  uint32_t allocChip;

  // lpn별로 매핑된 lane 수 (비어 있으면 항목 없음)
  std::unordered_map<uint64_t, uint32_t> mappedLanes;
  uint64_t mappedSectors;  // 매핑된 sector 총 수
  bool bSendToPAL;         // false이면 초기화 중이므로 PAL 접근과 통계를 생략

  // 페이지 유효 여부 조회에 쓰는 임시 버퍼 (매번 할당하지 않기 위함)
  Bitset scratchBits;
  std::vector<uint64_t> scratchLpns;

  // 같은 (블록, 페이지)에 대한 lane 요청을 ioFlag 하나로 묶은 NAND 요청
  struct PageOp {
    uint32_t pbn;
    uint32_t page;
    Bitset flag;
  };

  // 한 번의 동작이 만드는 NAND 요청 묶음. 읽기를 모두 같은 tick에 발행하고
  // 쓰기와 erase는 읽기가 끝난 뒤 발행해서 채널/chip 병렬성을 활용한다.
  struct Batch {
    std::vector<PageOp> reads;
    std::vector<PageOp> writes;
    std::unordered_map<uint64_t, size_t> readIndex;
    std::unordered_map<uint64_t, size_t> writeIndex;
    std::vector<uint32_t> erases;  // 지연된 erase 대상 블록
    uint64_t beginTick;
    uint64_t doneTick;
  } batch;

  struct {
    uint64_t switchMerge;
    uint64_t partialMerge;
    uint64_t fullMerge;
    uint64_t rwVictims;
    uint64_t hostWritePages;
    uint64_t sectorCopies;
    uint64_t nandWritePages;
    uint64_t eraseCount;
  } stat;

  std::vector<uint64_t> chipEraseCount;

  // 블록 관리
  uint32_t getFreeBlock();
  void insertFreeBlock(Block &&);
  void eraseBlock(uint32_t);
  void eraseBlockNow(uint32_t, uint64_t &);
  void beginBatch(uint64_t);
  void flushBatch();
  void queueOp(std::vector<PageOp> &, std::unordered_map<uint64_t, size_t> &,
               uint32_t, uint32_t, uint32_t);

  // sector 단위 조회와 쓰기
  bool isValid(uint32_t, uint32_t, uint32_t);
  bool findLatest(uint64_t, uint32_t, uint32_t &, uint32_t &);
  void invalidateLatest(uint64_t, uint32_t);
  void writeSector(uint32_t, uint32_t, uint64_t, uint32_t);
  void copySector(uint64_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
  void mapSector(uint64_t);
  void unmapSector(uint64_t);

  // FAST 동작
  void writeToLog(uint64_t, uint32_t);
  void writeToRW(uint64_t, uint32_t);
  void startSW(uint64_t);
  void appendSW(uint64_t, uint32_t);
  void mergeSW();
  void mergeLbn(uint64_t);
  void mergeRWVictim();

  void readInternal(Request &, uint64_t &);
  void writeInternal(Request &, uint64_t &);
  void trimInternal(Request &, uint64_t &);

  float calculateWearLeveling();

 public:
  FAST(ConfigReader &, Parameter &, PAL::PAL *, DRAM::AbstractDRAM *);
  ~FAST();

  bool initialize() override;

  void read(Request &, uint64_t &) override;
  void write(Request &, uint64_t &) override;
  void trim(Request &, uint64_t &) override;

  void format(LPNRange &, uint64_t &) override;

  Status *getStatus(uint64_t, uint64_t) override;

  void getStatList(std::vector<Stats> &, std::string) override;
  void getStatValues(std::vector<double> &) override;
  void resetStatValues() override;
};

}  // namespace FTL

}  // namespace SimpleSSD

#endif
