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

#include "ftl/fast.hh"

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <iterator>

#include "util/algorithm.hh"
#include "util/bitset.hh"

namespace SimpleSSD {

namespace FTL {

FAST::FAST(ConfigReader &c, Parameter &p, PAL::PAL *l, DRAM::AbstractDRAM *d)
    : AbstractFTL(p, l, d),
      pPAL(l),
      conf(c),
      allocChip(0),
      mappedSectors(0),
      bSendToPAL(true),
      scratchBits(p.ioUnitInPage) {
  maxRWLogs = conf.readUint(CONFIG_FTL, FTL_FAST_RW_LOG_BLOCKS);

  if (maxRWLogs == 0) {
    panic("ftl: FASTRWLogBlocks는 1 이상이어야 합니다");
  }

  rwGCPolicy = conf.readUint(CONFIG_FTL, FTL_FAST_RW_GC_POLICY);

  if (rwGCPolicy > 1) {
    panic("ftl: FASTRWGCPolicy는 0(FIFO) 또는 1(greedy)이어야 합니다");
  }

  rwChip = 0;
  bufferedInReq = 0;
  useWriteBuffer = conf.readUint(CONFIG_FTL, FTL_FAST_WRITE_BUFFER) == 1;

  // 열린 RW 로그 슬롯은 chip 수와 RW 로그 한도 중 작은 값만큼 둔다
  nOpenSlots = MIN(param.pageCountToMaxPerf, maxRWLogs);
  openRW.assign(nOpenSlots, -1);

  // 데이터 블록 전체 + SW 로그 1개 + RW 로그 + merge용 여유 블록 2개가 필요하다
  if (param.totalPhysicalBlocks <
      param.totalLogicalBlocks + maxRWLogs + 1 + 2) {
    panic("ftl: FAST를 위한 물리 블록이 부족합니다 (OverProvisioningRatio 확인)");
  }

  for (uint32_t i = 0; i < param.totalPhysicalBlocks; i++) {
    freeBlocks.emplace_back(Block(i, param.pagesInBlock, param.ioUnitInPage));
  }

  nFreeBlocks = param.totalPhysicalBlocks;

  status.totalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;

  memset(&sw, 0, sizeof(sw));
  memset(&stat, 0, sizeof(stat));

  swNext.assign(param.ioUnitInPage, 0);

  batch.beginTick = 0;
  batch.doneTick = 0;

  chipEraseCount.assign(param.pageCountToMaxPerf, 0);
  chipWrite.assign(param.pageCountToMaxPerf, 0);
  chanWrite.assign(param.ioUnitInPage, 0);
  chanErase.assign(param.ioUnitInPage, 0);

  dataBlockMap.reserve(param.totalLogicalBlocks);
}

FAST::~FAST() {
#ifdef FAST_VERIFY_VICTIM
  fprintf(stderr, "[FAST_VERIFY_VICTIM] policy=%u 검사한 victim 선택 %" PRIu64 "회, 모두 통과\n",
          rwGCPolicy, verifiedVictims);
#endif
}

bool FAST::initialize() {
  uint64_t nTotalLogicalPages;
  uint64_t nPagesToWarmup;
  uint64_t nPagesToInvalidate;
  uint64_t tick;
  FILLING_MODE mode;

  Request req(param.ioUnitInPage);

  debugprint(LOG_FTL_FAST, "Initialization started");

  nTotalLogicalPages = param.totalLogicalBlocks * param.pagesInBlock;
  nPagesToWarmup =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_FILL_RATIO);
  nPagesToInvalidate =
      nTotalLogicalPages * conf.readFloat(CONFIG_FTL, FTL_INVALID_PAGE_RATIO);
  mode = (FILLING_MODE)conf.readUint(CONFIG_FTL, FTL_FILLING_MODE);

  debugprint(LOG_FTL_FAST, "Total logical pages: %" PRIu64, nTotalLogicalPages);
  debugprint(LOG_FTL_FAST, "Total logical pages to fill: %" PRIu64,
             nPagesToWarmup);
  debugprint(LOG_FTL_FAST, "Total invalidated pages to create: %" PRIu64,
             nPagesToInvalidate);

  req.ioFlag.set();

  // 초기화 중에는 PAL에 요청을 보내지 않고 상태만 갱신한다
  bSendToPAL = false;

  // 1단계: 채우기
  if (mode == FILLING_MODE_0 || mode == FILLING_MODE_1) {
    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal(req, tick);
    }
  }
  else {
    std::mt19937_64 gen(FAST_RANDOM_SEED);
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToWarmup; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick);
    }
  }

  // 2단계: 무효 페이지 만들기
  if (mode == FILLING_MODE_0) {
    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = i;
      writeInternal(req, tick);
    }
  }
  else if (mode == FILLING_MODE_1) {
    std::mt19937_64 gen(FAST_RANDOM_SEED + 1);
    std::uniform_int_distribution<uint64_t> dist(
        0, nPagesToWarmup ? nPagesToWarmup - 1 : 0);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick);
    }
  }
  else {
    std::mt19937_64 gen(FAST_RANDOM_SEED + 1);
    std::uniform_int_distribution<uint64_t> dist(0, nTotalLogicalPages - 1);

    for (uint64_t i = 0; i < nPagesToInvalidate; i++) {
      tick = 0;
      req.lpn = dist(gen);
      writeInternal(req, tick);
    }
  }

  bSendToPAL = true;

  // 초기화 과정의 통계는 버린다
  resetStatValues();

  debugprint(LOG_FTL_FAST, "Mapped logical pages: %zu", mappedLanes.size());
  debugprint(LOG_FTL_FAST, "Initialization finished");

  return true;
}

void FAST::read(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  if (req.ioFlag.count() > 0) {
    readInternal(req, tick);

    debugprint(LOG_FTL_FAST,
               "READ  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
               ")",
               req.lpn, begin, tick, tick - begin);
  }
  else {
    warn("FTL got empty request");
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ);
}

void FAST::write(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  if (req.ioFlag.count() > 0) {
    writeInternal(req, tick);

    debugprint(LOG_FTL_FAST,
               "WRITE | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
               ")",
               req.lpn, begin, tick, tick - begin);
  }
  else {
    warn("FTL got empty request");
  }

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE);
}

void FAST::trim(Request &req, uint64_t &tick) {
  uint64_t begin = tick;

  trimInternal(req, tick);

  debugprint(LOG_FTL_FAST,
             "TRIM  | LPN %" PRIu64 " | %" PRIu64 " - %" PRIu64 " (%" PRIu64
             ")",
             req.lpn, begin, tick, tick - begin);

  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM);
}

void FAST::format(LPNRange &range, uint64_t &tick) {
  std::vector<uint64_t> lbns;

  beginBatch(tick);

  for (uint64_t lpn = range.slpn; lpn < range.slpn + range.nlp; lpn++) {
    for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
      uint32_t pbn;
      uint32_t page;
      uint64_t key = lpn * param.ioUnitInPage + lane;
      bool mapped = false;

      if (findLatest(lpn, lane, pbn, page)) {
        invalidateLatest(lpn, lane);

        lbns.push_back(lpn / param.pagesInBlock);

        mapped = true;
      }

      if (inBuffer(key)) {
        dropFromBuffer(key);

        mapped = true;
      }

      if (mapped) {
        unmapSector(lpn);
      }
    }
  }

  std::sort(lbns.begin(), lbns.end());
  lbns.erase(std::unique(lbns.begin(), lbns.end()), lbns.end());

  // 유효 sector가 하나도 남지 않은 데이터 블록과 SW 로그는 회수한다
  for (auto lbn : lbns) {
    auto data = dataBlockMap.find(lbn);

    if (data != dataBlockMap.end()) {
      if (blocks.at(data->second).getValidPageCount() == 0) {
        eraseBlock(data->second);
        dataBlockMap.erase(data);
      }
    }

    if (sw.valid && sw.lbn == lbn &&
        blocks.at(sw.pbn).getValidPageCount() == 0) {
      eraseBlock(sw.pbn);
      sw.valid = false;
    }
  }

  flushBatch();

  tick = batch.doneTick;
  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::FORMAT);
}

Status *FAST::getStatus(uint64_t lpnBegin, uint64_t lpnEnd) {
  status.freePhysicalBlocks = nFreeBlocks;

  if (lpnBegin == 0 && lpnEnd >= status.totalLogicalPages) {
    status.mappedLogicalPages = mappedLanes.size();
  }
  else {
    status.mappedLogicalPages = 0;

    for (uint64_t lpn = lpnBegin; lpn < lpnEnd; lpn++) {
      if (mappedLanes.count(lpn) > 0) {
        status.mappedLogicalPages++;
      }
    }
  }

  return &status;
}

/* ---------------------------- 블록 관리 ---------------------------- */

// 비어 있는 블록 중 하나를 꺼내 사용 중 블록으로 옮긴다.
// 병렬성을 위해 chip(blockIdx % pageCountToMaxPerf)을 돌아가며 고른다.
uint32_t FAST::getFreeBlock() {
  uint32_t chip = allocChip;

  allocChip = (allocChip + 1) % param.pageCountToMaxPerf;

  return getFreeBlockOnChip(chip);
}

// 지정한 chip의 빈 블록을 꺼낸다. 그 chip에 빈 블록이 없으면 맨 앞 블록을 쓴다.
uint32_t FAST::getFreeBlockOnChip(uint32_t chip) {
  uint32_t blockIndex = 0;

  // 지연된 erase가 있으면 먼저 반영해서 여유 블록을 확보한다
  if (nFreeBlocks == 0 && !batch.erases.empty()) {
    flushBatch();
  }

  if (nFreeBlocks == 0) {
    panic("ftl: No free block left");
  }

  auto iter = freeBlocks.begin();

  for (; iter != freeBlocks.end(); iter++) {
    if (iter->getBlockIndex() % param.pageCountToMaxPerf == chip) {
      break;
    }
  }

  // 해당 chip의 빈 블록이 없으면 맨 앞 블록을 쓴다
  if (iter == freeBlocks.end()) {
    iter = freeBlocks.begin();
  }

  blockIndex = iter->getBlockIndex();

  if (blocks.find(blockIndex) != blocks.end()) {
    panic("Corrupted");
  }

  blocks.emplace(blockIndex, std::move(*iter));
  freeBlocks.erase(iter);
  nFreeBlocks--;

  return blockIndex;
}

// erase 횟수 오름차순을 유지하며 빈 블록 목록에 넣는다 (마모 균등화)
void FAST::insertFreeBlock(Block &&block) {
  uint32_t count = block.getEraseCount();
  auto pos = freeBlocks.end();

  while (pos != freeBlocks.begin()) {
    auto prev = std::prev(pos);

    if (prev->getEraseCount() <= count) {
      break;
    }

    pos = prev;
  }

  freeBlocks.emplace(pos, std::move(block));
  nFreeBlocks++;
}

// 블록의 erase를 batch 마지막으로 미룬다
void FAST::eraseBlock(uint32_t pbn) {
  batch.erases.push_back(pbn);
}

// 실제 erase 수행: 블록 상태를 지우고 빈 블록 목록으로 돌려보낸다
void FAST::eraseBlockNow(uint32_t pbn, uint64_t &tick) {
  static uint64_t threshold = conf.readUint(CONFIG_FTL, FTL_BAD_BLOCK_THRESHOLD);
  auto block = blocks.find(pbn);

  if (block == blocks.end()) {
    panic("No such block");
  }

  if (block->second.getValidPageCount() != 0) {
    panic("There are valid pages in victim block");
  }

  block->second.erase();

  if (bSendToPAL) {
    PAL::Request req(param.ioUnitInPage);

    req.reqID = 0;
    req.reqSubID = 0;
    req.blockIndex = pbn;
    req.pageIndex = 0;
    req.ioFlag.set();

    pPAL->erase(req, tick);

    stat.eraseCount++;
    chipEraseCount.at(pbn % param.pageCountToMaxPerf)++;

    // 슈퍼블록 erase는 채널마다 물리 블록 1개씩을 지운다
    for (auto &iter : chanErase) {
      iter++;
    }

    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::ERASE_INTERNAL);
  }

  // erase 한도를 넘은 블록은 불량 블록으로 간주하고 되돌려 놓지 않는다
  if (block->second.getEraseCount() < threshold) {
    insertFreeBlock(std::move(block->second));
  }

  blocks.erase(block);
}

void FAST::beginBatch(uint64_t tick) {
  batch.beginTick = tick;
  batch.doneTick = tick;
}

// 같은 (블록, 페이지)에 대한 lane 요청은 ioFlag 하나로 묶는다
void FAST::queueOp(std::vector<PageOp> &ops,
                   std::unordered_map<uint64_t, size_t> &index, uint32_t pbn,
                   uint32_t page, uint32_t lane) {
  uint64_t key = ((uint64_t)pbn << 32) | page;
  auto iter = index.find(key);

  if (iter == index.end()) {
    PageOp op = {pbn, page, Bitset(param.ioUnitInPage)};

    op.flag.reset();
    ops.push_back(op);

    iter = index.emplace(key, ops.size() - 1).first;
  }

  ops.at(iter->second).flag.set(lane);
}

// 모아 둔 NAND 요청을 발행한다.
// 읽기는 모두 같은 tick에 발행하고, 쓰기와 erase는 읽기가 끝난 뒤 발행한다.
// 슈퍼블록 요청이므로 PAL에서 lane(채널)별로 병렬 처리된다.
void FAST::flushBatch() {
  uint64_t readDone = batch.beginTick;
  uint64_t writeDone;
  uint64_t eraseDone;
  uint64_t t;

  if (bSendToPAL) {
    PAL::Request req(param.ioUnitInPage);

    req.reqID = 0;
    req.reqSubID = 0;

    for (auto &iter : batch.reads) {
      req.blockIndex = iter.pbn;
      req.pageIndex = iter.page;
      req.ioFlag = iter.flag;

      t = batch.beginTick;

      pPAL->read(req, t);

      readDone = MAX(readDone, t);
    }
  }

  writeDone = readDone;
  eraseDone = readDone;

  if (bSendToPAL) {
    PAL::Request req(param.ioUnitInPage);

    req.reqID = 0;
    req.reqSubID = 0;

    for (auto &iter : batch.writes) {
      req.blockIndex = iter.pbn;
      req.pageIndex = iter.page;
      req.ioFlag = iter.flag;

      t = readDone;

      pPAL->write(req, t);

      writeDone = MAX(writeDone, t);
    }
  }

  // erase 중 빈 블록 목록이 바뀌므로 목록을 옮겨 놓고 처리한다
  std::vector<uint32_t> erases;

  erases.swap(batch.erases);

  for (auto pbn : erases) {
    t = readDone;

    eraseBlockNow(pbn, t);

    eraseDone = MAX(eraseDone, t);
  }

  batch.reads.clear();
  batch.writes.clear();
  batch.readIndex.clear();
  batch.writeIndex.clear();

  batch.doneTick = MAX(writeDone, eraseDone);
  batch.beginTick = batch.doneTick;
}

/* -------------------- sector 단위 조회와 쓰기 -------------------- */

// 블록의 (page, lane) 칸이 유효한지 확인한다
bool FAST::isValid(uint32_t pbn, uint32_t page, uint32_t lane) {
  auto block = blocks.find(pbn);

  if (block == blocks.end()) {
    return false;
  }

  scratchBits.reset();

  if (!block->second.getPageInfo(page, scratchLpns, scratchBits)) {
    return false;
  }

  return scratchBits.test(lane);
}

// (lpn, lane)의 NAND 최신 버전 위치를 찾는다: RW 로그 -> SW 로그 -> 데이터 블록 순.
// physLane은 실제로 놓인 lane이다 (데이터 블록/SW 로그에서는 항상 lane과 같다).
// 쓰기 버퍼에 있는 버전은 여기서 찾지 않는다 (inBuffer로 따로 확인).
bool FAST::findLatest(uint64_t lpn, uint32_t lane, uint32_t &pbn,
                      uint32_t &page, uint32_t &physLane) {
  uint64_t lbn = lpn / param.pagesInBlock;
  uint32_t offset = lpn % param.pagesInBlock;

  auto rw = rwMap.find(lpn * param.ioUnitInPage + lane);

  if (rw != rwMap.end()) {
    pbn = rw->second.pbn;
    page = rw->second.page;
    physLane = rw->second.lane;

    return true;
  }

  physLane = lane;

  if (sw.valid && sw.lbn == lbn && offset < swNext[lane] &&
      isValid(sw.pbn, offset, lane)) {
    pbn = sw.pbn;
    page = offset;

    return true;
  }

  auto data = dataBlockMap.find(lbn);

  if (data != dataBlockMap.end() && isValid(data->second, offset, lane)) {
    pbn = data->second;
    page = offset;

    return true;
  }

  return false;
}

bool FAST::findLatest(uint64_t lpn, uint32_t lane, uint32_t &pbn,
                      uint32_t &page) {
  uint32_t physLane;

  return findLatest(lpn, lane, pbn, page, physLane);
}

// (lpn, lane)의 현재 최신 버전을 무효화하고 RW 섹터 맵에서도 지운다
void FAST::invalidateLatest(uint64_t lpn, uint32_t lane) {
  uint32_t pbn;
  uint32_t page;
  uint32_t physLane;

  if (findLatest(lpn, lane, pbn, page, physLane)) {
    blocks.at(pbn).invalidate(page, physLane);

    rwMap.erase(lpn * param.ioUnitInPage + lane);
  }
}

// sector 하나를 블록의 (page, lane) 칸에 쓴다. lane은 실제로 쓸 lane이다.
// value는 칸에 저장할 값으로, 데이터 블록/SW 로그는 lpn, RW 로그는 sector 키다.
void FAST::writeSector(uint32_t pbn, uint32_t page, uint64_t value,
                       uint32_t lane) {
  auto block = blocks.find(pbn);

  if (block == blocks.end()) {
    panic("ftl: 사용 중이지 않은 블록에 쓰기를 시도했습니다");
  }

  block->second.write(page, value, lane, batch.beginTick);

  if (bSendToPAL) {
    queueOp(batch.writes, batch.writeIndex, pbn, page, lane);

    stat.nandWritePages++;
    chanWrite.at(lane)++;
    chipWrite.at(pbn % param.pageCountToMaxPerf)++;
  }
}

// merge 중 유효 sector를 다른 블록으로 옮긴다 (읽기 + 쓰기 + 원본 무효화).
// 원본은 실제 lane(srcLane)에서 읽고, 대상(데이터 블록/SW 로그)에는 논리 lane에 쓴다.
void FAST::copySector(uint64_t lpn, uint32_t lane, uint32_t srcBlock,
                      uint32_t srcPage, uint32_t srcLane, uint32_t dstBlock,
                      uint32_t dstPage) {
  if (bSendToPAL) {
    queueOp(batch.reads, batch.readIndex, srcBlock, srcPage, srcLane);

    stat.sectorCopies++;
  }

  blocks.at(srcBlock).invalidate(srcPage, srcLane);

  rwMap.erase(lpn * param.ioUnitInPage + lane);

  writeSector(dstBlock, dstPage, lpn, lane);
}

void FAST::mapSector(uint64_t lpn) {
  mappedLanes[lpn]++;
  mappedSectors++;
}

void FAST::unmapSector(uint64_t lpn) {
  auto iter = mappedLanes.find(lpn);

  if (iter == mappedLanes.end()) {
    panic("ftl: 매핑되지 않은 sector를 해제하려 했습니다");
  }

  if (--(iter->second) == 0) {
    mappedLanes.erase(iter);
  }

  mappedSectors--;
}

/* --------------------------- FAST 동작 --------------------------- */

// SW 로그를 새로 시작한다
void FAST::startSW(uint64_t lbn) {
  sw.valid = true;
  sw.pbn = getFreeBlock();
  sw.lbn = lbn;

  std::fill(swNext.begin(), swNext.end(), 0);
}

// SW 로그의 해당 lane에 이어서 쓴다 (offset == swNext[lane]이어야 한다)
void FAST::appendSW(uint64_t lpn, uint32_t lane) {
  // 버퍼에 있던 이전 버전은 이 쓰기로 대체된다
  dropFromBuffer(lpn * param.ioUnitInPage + lane);
  invalidateLatest(lpn, lane);
  writeSector(sw.pbn, swNext[lane], lpn, lane);

  swNext[lane]++;
}

// SW 로그를 데이터 블록과 합친다.
//  - switch : 모든 lane이 가득 차고 모두 유효하면 SW 블록이 그대로 데이터 블록이 된다
//  - partial: SW에 쓴 부분이 모두 유효하면 남은 칸만 데이터 블록에서 복사한다
//  - full   : 그 외에는 새 블록에 모아서 합친다
void FAST::mergeSW() {
  if (!sw.valid) {
    return;
  }

  uint64_t lbn = sw.lbn;
  bool prefixValid = true;
  bool allFull = true;

  for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
    if (swNext[lane] != param.pagesInBlock) {
      allFull = false;
    }

    for (uint32_t o = 0; prefixValid && o < swNext[lane]; o++) {
      if (!isValid(sw.pbn, o, lane)) {
        prefixValid = false;
      }
    }
  }

  if (!prefixValid) {
    mergeLbn(lbn);
  }
  else {
    auto data = dataBlockMap.find(lbn);
    bool hasOld = data != dataBlockMap.end();
    uint32_t oldPbn = hasOld ? data->second : 0;

    if (allFull) {
      stat.switchMerge++;
    }
    else {
      // partial merge: 최신 버전이 기존 데이터 블록에 있는 칸만 복사한다
      for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
        for (uint32_t o = swNext[lane]; o < param.pagesInBlock; o++) {
          uint64_t lpn = lbn * param.pagesInBlock + o;
          uint32_t pbn;
          uint32_t page;
          uint32_t physLane;

          // 기존 데이터 블록에 있는 칸만 대상이므로 physLane은 lane과 같다
          if (findLatest(lpn, lane, pbn, page, physLane) && hasOld &&
              pbn == oldPbn) {
            copySector(lpn, lane, pbn, page, physLane, sw.pbn, o);
          }
        }
      }

      stat.partialMerge++;
    }

    // SW 블록이 새 데이터 블록이 되고, 기존 데이터 블록은 비워서 회수한다
    dataBlockMap[lbn] = sw.pbn;

    if (hasOld) {
      eraseBlock(oldPbn);
    }

    sw.valid = false;
  }

  if (bSendToPAL) {
    batch.doneTick +=
        applyLatency(CPU::FTL__PAGE_MAPPING, CPU::DO_GARBAGE_COLLECTION);
    batch.beginTick = batch.doneTick;
  }

  flushBatch();
}

// 논리 블록 하나의 모든 최신 sector를 새 데이터 블록으로 모은다 (full merge).
// RW 로그/SW 로그/기존 데이터 블록의 최신 버전을 모두 가져오므로, 이후 해당
// 논리 블록의 sector는 RW 로그에 남지 않는다.
void FAST::mergeLbn(uint64_t lbn) {
  uint32_t newPbn = getFreeBlock();
  uint32_t copied = 0;

  for (uint32_t o = 0; o < param.pagesInBlock; o++) {
    uint64_t lpn = lbn * param.pagesInBlock + o;

    for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
      uint32_t pbn;
      uint32_t page;
      uint32_t physLane;

      if (findLatest(lpn, lane, pbn, page, physLane)) {
        copySector(lpn, lane, pbn, page, physLane, newPbn, o);

        copied++;
      }
    }
  }

  auto data = dataBlockMap.find(lbn);

  if (data != dataBlockMap.end()) {
    eraseBlock(data->second);
    dataBlockMap.erase(data);
  }

  // 이 논리 블록의 SW 로그 sector도 모두 옮겨졌으므로 SW 블록을 회수한다
  if (sw.valid && sw.lbn == lbn) {
    eraseBlock(sw.pbn);
    sw.valid = false;
  }

  if (copied > 0) {
    dataBlockMap[lbn] = newPbn;
  }
  else {
    // 옮길 sector가 없었으면 쓰지 않은 블록을 그대로 돌려준다
    auto block = blocks.find(newPbn);

    insertFreeBlock(std::move(block->second));
    blocks.erase(block);
  }

  stat.fullMerge++;
}

// RW 로그 중 비울 victim을 정책에 따라 고른다. SW 로그는 블록이 1개뿐이라
// 고를 대상이 없으므로 이 정책은 RW 로그에만 적용된다.
//  - 0 (FIFO): 가장 오래된 블록
//  - 1 (greedy): 유효 sector가 가장 적은 블록. 같으면 더 오래된 블록을 고른다.
//    유효 sector가 적을수록 merge로 옮길 sector가 적을 것이라는 기준이다.
// 쓰기 버퍼를 쓰면 지금 채우고 있는(열린) RW 로그는 후보에서 뺀다. 그래서 victim은
// 모든 채널이 다 찬 RW 로그 중에서만 고른다.
std::list<uint32_t>::iterator FAST::selectRWVictim() {
  auto best = rwLogs.end();
  uint32_t bestValid = 0;

  for (auto iter = rwLogs.begin(); iter != rwLogs.end(); ++iter) {
    if (isOpenRW(*iter)) {
      continue;
    }

    if (rwGCPolicy == 0) {
      return iter;
    }

    uint32_t valid = blocks.at(*iter).getValidPageCountRaw();

    if (best == rwLogs.end() || valid < bestValid) {
      best = iter;
      bestValid = valid;
    }
  }

  return best;
}

#ifdef FAST_VERIFY_VICTIM
// 정책이 고른 victim이 정책 정의와 맞는지 검사한다 (빌드 옵션 -DFAST_VERIFY_VICTIM).
// 블록의 유효 sector 수는 Block이 아니라 rwMap에서 따로 세어 대조한다.
void FAST::verifyRWVictim(std::list<uint32_t>::iterator sel) {
  std::unordered_map<uint32_t, uint32_t> validByMap;

  for (auto &e : rwMap) {
    validByMap[e.second.pbn]++;
  }

  for (auto pbn : rwLogs) {
    if (blocks.at(pbn).getValidPageCountRaw() != validByMap[pbn]) {
      panic("ftl: RW 로그 %u의 유효 sector 수 불일치 (블록 %u, rwMap %u)", pbn,
            blocks.at(pbn).getValidPageCountRaw(), validByMap[pbn]);
    }
  }

  // 열린 RW 로그를 뺀 블록 중에서 정책 정의대로 고른 결과를 따로 구한다
  auto expect = rwLogs.end();

  for (auto iter = rwLogs.begin(); iter != rwLogs.end(); ++iter) {
    if (isOpenRW(*iter)) {
      continue;
    }

    if (expect == rwLogs.end() ||
        (rwGCPolicy == 1 && validByMap[*iter] < validByMap[*expect])) {
      expect = iter;
    }
  }

  if (expect == rwLogs.end() || sel == rwLogs.end()) {
    panic("ftl: victim 후보가 없습니다");
  }

  // 쓰기 버퍼 모드에서는 victim이 모든 lane에서 꽉 찬 블록이어야 한다
  if (useWriteBuffer && !isRWFull(*sel)) {
    panic("ftl: 다 차지 않은 RW 로그 %u를 victim으로 골랐습니다", *sel);
  }

  if (sel != expect) {
    panic("ftl: victim 선택 오류: 선택 %u (유효 %u), 기대 %u (유효 %u)", *sel,
          validByMap[*sel], *expect, validByMap[*expect]);
  }

  verifiedVictims++;
}
#endif

// 정책이 고른 RW 로그 블록을 victim으로 비운다.
// victim에 유효 sector를 가진 모든 논리 블록을 각각 full merge 한다.
// 여러 논리 블록의 merge 요청을 하나의 batch로 모아 채널/chip 병렬성을 쓴다.
void FAST::mergeRWVictim() {
  auto victimIter = selectRWVictim();
#ifdef FAST_VERIFY_VICTIM
  verifyRWVictim(victimIter);
#endif
  if (victimIter == rwLogs.end()) {
    panic("ftl: RW 로그 victim 후보가 없습니다");
  }

  uint32_t victim = *victimIter;
  std::vector<uint64_t> unique;
  std::vector<uint64_t> lpns;
  Bitset bit(param.ioUnitInPage);

  auto &block = blocks.at(victim);

  // victim의 유효 sector가 속한 논리 블록을 등장 순서대로 모은다
  for (uint32_t page = 0; page < param.pagesInBlock; page++) {
    if (block.getPageInfo(page, lpns, bit)) {
      for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
        if (bit.test(lane)) {
          // RW 로그 칸에는 sector 키(lpn * lane 수 + 논리 lane)가 저장되어 있다
          uint64_t lbn =
              lpns.at(lane) / param.ioUnitInPage / param.pagesInBlock;

          if (std::find(unique.begin(), unique.end(), lbn) == unique.end()) {
            unique.push_back(lbn);
          }
        }
      }
    }
  }

  for (auto lbn : unique) {
    mergeLbn(lbn);
  }

  // 이제 victim에는 유효 sector가 없다
  rwLogs.erase(victimIter);
  eraseBlock(victim);

  stat.rwVictims++;

  if (bSendToPAL) {
    batch.doneTick +=
        applyLatency(CPU::FTL__PAGE_MAPPING, CPU::DO_GARBAGE_COLLECTION);
    batch.beginTick = batch.doneTick;
  }

  flushBatch();
}

// RW 로그에 쓴다. 해당 lane에 빈 칸이 있는 가장 오래된 블록을 쓰고, 없으면 새
// 블록을 받는다. RW 로그가 한도만큼 있으면 가장 앞 블록을 먼저 merge 한다.
void FAST::writeToRW(uint64_t lpn, uint32_t lane) {
  uint32_t target = 0;
  bool found = false;

  for (auto pbn : rwLogs) {
    if (blocks.at(pbn).getNextWritePageIndex(lane) < param.pagesInBlock) {
      target = pbn;
      found = true;

      break;
    }
  }

  if (!found) {
    if (rwLogs.size() >= maxRWLogs) {
      mergeRWVictim();
    }

    rwLogs.push_back(getFreeBlock());

    target = rwLogs.back();
  }

  uint32_t page = blocks.at(target).getNextWritePageIndex(lane);
  uint64_t key = lpn * param.ioUnitInPage + lane;

  invalidateLatest(lpn, lane);
  writeSector(target, page, key, lane);

  rwMap[key] = {target, page, lane};
}

// RW 로그로 가는 쓰기: 쓰기 버퍼를 쓰면 버퍼에 넣고, 아니면 바로 RW 로그에 쓴다
void FAST::writeRW(uint64_t lpn, uint32_t lane) {
  if (useWriteBuffer) {
    bufferSector(lpn, lane);
  }
  else {
    writeToRW(lpn, lane);
  }
}

/* --------------------------- 쓰기 버퍼 --------------------------- */

bool FAST::inBuffer(uint64_t key) {
  return wbufIndex.count(key) > 0;
}

// 지금 어떤 슬롯에서 채우고 있는 RW 로그인지 확인한다
bool FAST::isOpenRW(uint32_t pbn) {
  for (auto iter : openRW) {
    if (iter == (int64_t)pbn) {
      return true;
    }
  }

  return false;
}

// 모든 lane의 쓰기 포인터가 블록 끝에 도달했는지 확인한다
bool FAST::isRWFull(uint32_t pbn) {
  auto &block = blocks.at(pbn);

  for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
    if (block.getNextWritePageIndex(lane) < param.pagesInBlock) {
      return false;
    }
  }

  return true;
}

// 업데이트 쓰기를 버퍼에 넣는다. 같은 sector가 이미 있으면 덮어쓰기만 하고
// NAND에는 쓰지 않는다. 버퍼가 lane 수만큼 차면 슈퍼페이지 1개로 내보낸다.
void FAST::bufferSector(uint64_t lpn, uint32_t lane) {
  uint64_t key = lpn * param.ioUnitInPage + lane;

  if (inBuffer(key)) {
    if (bSendToPAL) {
      stat.bufferAbsorbed++;
    }

    return;
  }

  // 최신 버전이 버퍼로 옮겨졌으므로 NAND의 이전 버전은 지금 무효화한다.
  // 그래야 버퍼에 있는 동안 merge가 낡은 버전을 옮기지 않는다.
  invalidateLatest(lpn, lane);

  wbufIndex.emplace(key, wbuf.size());
  wbuf.push_back(key);

  bufferedInReq++;

  if (wbuf.size() >= param.ioUnitInPage) {
    flushWriteBuffer();
  }
}

// 버퍼에서 sector를 뺀다 (더 새 버전이 다른 곳에 쓰였거나 trim/format된 경우).
// 순서는 중요하지 않으므로 마지막 항목을 빈자리로 옮긴다.
void FAST::dropFromBuffer(uint64_t key) {
  auto iter = wbufIndex.find(key);

  if (iter == wbufIndex.end()) {
    return;
  }

  size_t pos = iter->second;
  uint64_t last = wbuf.back();

  wbuf[pos] = last;
  wbufIndex[last] = pos;

  wbuf.pop_back();
  wbufIndex.erase(key);
}

// 슬롯의 열린 RW 로그를 돌려준다. 없거나 모든 lane이 다 찼으면 닫고 새 블록을
// 연다. 이때 RW 로그가 한도만큼 있으면 다 찬 RW 로그 중 victim을 merge/erase 한다.
uint32_t FAST::ensureOpenRW(uint32_t slot) {
  int64_t cur = openRW.at(slot);

  if (cur >= 0 && !isRWFull((uint32_t)cur)) {
    return (uint32_t)cur;
  }

  // 다 찬 블록은 닫아서 victim 후보가 되게 한다
  openRW.at(slot) = -1;

  if (rwLogs.size() >= maxRWLogs) {
    mergeRWVictim();
  }

  uint32_t pbn = getFreeBlockOnChip(slot % param.pageCountToMaxPerf);

  rwLogs.push_back(pbn);
  openRW.at(slot) = (int64_t)pbn;

  return pbn;
}

// 버퍼의 sector를 지금 슬롯의 RW 로그에 쓴다. sector마다 쓰기 포인터가 가장 낮은
// lane을 고르므로, 버퍼가 꽉 찼을 때는 같은 페이지의 모든 lane(채널)에 한 번에
// 쓰이고 queueOp가 이를 NAND 요청 1개로 묶는다.
void FAST::flushWriteBuffer() {
  if (wbuf.empty()) {
    return;
  }

  std::vector<uint64_t> keys;

  // merge 중에도 버퍼 상태가 일관되도록 먼저 비운다
  keys.swap(wbuf);
  wbufIndex.clear();

  uint32_t slot = rwChip;

  rwChip = (rwChip + 1) % nOpenSlots;

  // 이번 flush가 만든 NAND 쓰기 요청 (같은 블록/페이지의 lane은 한 요청으로 묶인다)
  std::vector<PageOp> flushOps;

  for (auto key : keys) {
    uint32_t target = ensureOpenRW(slot);
    auto &block = blocks.at(target);
    uint32_t physLane = 0;
    uint32_t page = param.pagesInBlock;

    for (uint32_t l = 0; l < param.ioUnitInPage; l++) {
      uint32_t next = block.getNextWritePageIndex(l);

      if (next < page) {
        page = next;
        physLane = l;
      }
    }

    // NAND의 이전 버전은 버퍼에 넣을 때 이미 무효화했다
    writeSector(target, page, key, physLane);

    rwMap[key] = {target, page, physLane};

    if (flushOps.empty() || flushOps.back().pbn != target ||
        flushOps.back().page != page) {
      PageOp op = {target, page, Bitset(param.ioUnitInPage)};

      op.flag.reset();
      flushOps.push_back(op);
    }

    flushOps.back().flag.set(physLane);
  }

  if (bSendToPAL) {
    stat.bufferFlushes++;
    stat.bufferWriteOps += flushOps.size();

    // 확인용 로그: lane 비트가 모두 1이면 슈퍼페이지 1개로 모든 채널에 쓴 것이다.
    // chip은 블록 번호 % chip 수이며, 슬롯과 다르면 그 chip에 빈 블록이 없었던 것이다.
    for (auto &op : flushOps) {
      std::string lanes;

      for (uint32_t l = 0; l < param.ioUnitInPage; l++) {
        lanes += op.flag.test(l) ? '1' : '0';
      }

      debugprint(LOG_FTL_FAST,
                 "FLUSH | slot %u | chip %u | block %u | page %u | lanes %s",
                 slot, op.pbn % param.pageCountToMaxPerf, op.pbn, op.page,
                 lanes.c_str());
    }
  }
}

// 데이터 블록에서 충돌이 난 sector를 로그 블록으로 보낸다 (논문 Algorithm 2)
void FAST::writeToLog(uint64_t lpn, uint32_t lane) {
  uint64_t lbn = lpn / param.pagesInBlock;
  uint32_t offset = lpn % param.pagesInBlock;
  bool sameLbn = sw.valid && sw.lbn == lbn;

  if (offset == 0) {
    if (!(sameLbn && swNext[lane] == 0)) {
      // 새 순차 쓰기의 시작: 기존 SW 로그를 정리하고 새 SW 로그를 시작한다
      mergeSW();
      startSW(lbn);
    }

    appendSW(lpn, lane);
  }
  else if (sameLbn) {
    if (offset == swNext[lane]) {
      // 순차 쓰기가 이어지는 경우
      appendSW(lpn, lane);
    }
    else {
      // 순차성이 깨졌으므로 SW를 합치고 이번 쓰기는 RW 로그로 보낸다
      mergeSW();
      writeRW(lpn, lane);
    }
  }
  else {
    writeRW(lpn, lane);
  }
}

/* ----------------------------- 읽기/쓰기 ----------------------------- */

void FAST::readInternal(Request &req, uint64_t &tick) {
  PAL::Request palRequest(req);
  std::vector<PageOp> ops;
  std::unordered_map<uint64_t, size_t> index;
  uint64_t beginAt = tick;
  uint64_t finishedAt = tick;
  uint32_t pbn;
  uint32_t page;
  uint32_t physLane;
  uint32_t buffered = 0;

  // 매핑 테이블(DRAM) 조회
  pDRAM->read(&dataBlockMap, 8, beginAt);

  for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
    if (!req.ioFlag.test(lane)) {
      continue;
    }

    // 쓰기 버퍼에 있는 최신 버전은 DRAM에서 읽는다
    if (inBuffer(req.lpn * param.ioUnitInPage + lane)) {
      buffered++;
    }
    else if (findLatest(req.lpn, lane, pbn, page, physLane)) {
      blocks.at(pbn).read(page, physLane, beginAt);

      queueOp(ops, index, pbn, page, physLane);
    }
  }

  if (buffered > 0) {
    uint64_t t = beginAt;

    pDRAM->read(&wbuf, (uint64_t)buffered * (param.pageSize / param.ioUnitInPage),
                t);

    finishedAt = MAX(finishedAt, t);
  }

  if (ops.empty()) {
    if (buffered > 0) {
      tick = finishedAt;
    }

    return;
  }

  // lane별로 모은 요청을 같은 tick에 발행한다 (채널 병렬)
  for (auto &iter : ops) {
    uint64_t t = beginAt;

    palRequest.blockIndex = iter.pbn;
    palRequest.pageIndex = iter.page;
    palRequest.ioFlag = iter.flag;

    pPAL->read(palRequest, t);

    finishedAt = MAX(finishedAt, t);
  }

  tick = finishedAt;
  tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::READ_INTERNAL);
}

void FAST::writeInternal(Request &req, uint64_t &tick) {
  uint64_t lpn = req.lpn;
  uint64_t lbn = lpn / param.pagesInBlock;
  uint32_t offset = lpn % param.pagesInBlock;

  if (lbn >= param.totalLogicalBlocks) {
    panic("ftl: LPN %" PRIu64 " is out of range", lpn);
  }

  if (bSendToPAL) {
    stat.hostWritePages += req.ioFlag.count();

    pDRAM->read(&dataBlockMap, 8, tick);
    pDRAM->write(&dataBlockMap, 8, tick);
  }

  beginBatch(tick);

  bufferedInReq = 0;

  // 요청에 포함된 lane을 sector 단위로 하나씩 처리한다.
  // lane마다 쓰기 포인터가 독립이므로 같은 페이지의 다른 lane은 충돌이 아니다.
  for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
    if (!req.ioFlag.test(lane)) {
      continue;
    }

    uint32_t pbn;
    uint32_t page;
    uint32_t dbn;
    bool collision = false;

    if (!findLatest(lpn, lane, pbn, page) &&
        !inBuffer(lpn * param.ioUnitInPage + lane)) {
      mapSector(lpn);
    }

    // merge가 데이터 블록을 바꿀 수 있으므로 매번 다시 찾는다
    auto data = dataBlockMap.find(lbn);

    if (data == dataBlockMap.end()) {
      // 처음 쓰는 논리 블록: 데이터 블록을 할당한다
      dbn = getFreeBlock();
      dataBlockMap[lbn] = dbn;
    }
    else {
      dbn = data->second;

      // 이미 쓴 칸이거나 lane의 쓰기 포인터보다 앞이면 충돌 (NAND는 순차 쓰기만 가능)
      collision = offset < blocks.at(dbn).getNextWritePageIndex(lane);
    }

    if (!collision) {
      dropFromBuffer(lpn * param.ioUnitInPage + lane);
      invalidateLatest(lpn, lane);
      writeSector(dbn, offset, lpn, lane);
    }
    else {
      writeToLog(lpn, lane);
    }
  }

  // 버퍼에 넣은 sector는 DRAM 쓰기 지연만 받는다
  uint64_t bufferDone = tick;

  if (bSendToPAL && bufferedInReq > 0) {
    pDRAM->write(&wbuf,
                 (uint64_t)bufferedInReq * (param.pageSize / param.ioUnitInPage),
                 bufferDone);
  }

  flushBatch();

  if (bSendToPAL) {
    // 이번 요청이 버퍼를 flush했으면 NAND(와 GC)가 끝날 때까지 기다린다
    tick = MAX(batch.doneTick, bufferDone);
    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::WRITE_INTERNAL);
  }
}

void FAST::trimInternal(Request &req, uint64_t &tick) {
  bool trimmed = false;
  uint32_t pbn;
  uint32_t page;

  // trim은 요청의 ioFlag와 관계없이 lpn의 모든 lane을 지운다
  for (uint32_t lane = 0; lane < param.ioUnitInPage; lane++) {
    uint64_t key = req.lpn * param.ioUnitInPage + lane;
    bool mapped = false;

    if (findLatest(req.lpn, lane, pbn, page)) {
      invalidateLatest(req.lpn, lane);

      mapped = true;
    }

    if (inBuffer(key)) {
      dropFromBuffer(key);

      mapped = true;
    }

    if (mapped) {
      unmapSector(req.lpn);

      trimmed = true;
    }
  }

  if (trimmed) {
    if (bSendToPAL) {
      pDRAM->read(&dataBlockMap, 8, tick);
    }

    tick += applyLatency(CPU::FTL__PAGE_MAPPING, CPU::TRIM_INTERNAL);
  }
}

/* ------------------------------ 통계 ------------------------------ */

float FAST::calculateWearLeveling() {
  uint64_t totalEraseCnt = 0;
  uint64_t sumOfSquaredEraseCnt = 0;
  uint64_t numOfBlocks = param.totalLogicalBlocks;
  uint64_t eraseCnt;

  for (auto &iter : blocks) {
    eraseCnt = iter.second.getEraseCount();
    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  // freeBlocks는 erase 횟수 순으로 정렬되어 있으므로 뒤에서부터 0이 나올 때까지 센다
  for (auto riter = freeBlocks.rbegin(); riter != freeBlocks.rend(); riter++) {
    eraseCnt = riter->getEraseCount();

    if (eraseCnt == 0) {
      break;
    }

    totalEraseCnt += eraseCnt;
    sumOfSquaredEraseCnt += eraseCnt * eraseCnt;
  }

  if (sumOfSquaredEraseCnt == 0) {
    return -1;  // 마모 균등화를 판단할 수 없음
  }

  return (float)totalEraseCnt * totalEraseCnt /
         (numOfBlocks * sumOfSquaredEraseCnt);
}

void FAST::getStatList(std::vector<Stats> &list, std::string prefix) {
  Stats temp;

  temp.name = prefix + "fast.merge.switch";
  temp.desc = "Total switch merge count";
  list.push_back(temp);

  temp.name = prefix + "fast.merge.partial";
  temp.desc = "Total partial merge count (SW log)";
  list.push_back(temp);

  temp.name = prefix + "fast.merge.full";
  temp.desc = "Total full merge count (per logical block)";
  list.push_back(temp);

  temp.name = prefix + "fast.rw_victims";
  temp.desc = "Total RW log block victim count";
  list.push_back(temp);

  temp.name = prefix + "fast.merge.sector_copies";
  temp.desc = "Total copied sectors (I/O units) during merge";
  list.push_back(temp);

  temp.name = prefix + "fast.host.write_pages";
  temp.desc = "Total host write pages (I/O units)";
  list.push_back(temp);

  temp.name = prefix + "fast.nand_write_pages";
  temp.desc = "Total pages (I/O units) programmed to NAND";
  list.push_back(temp);

  temp.name = prefix + "fast.waf";
  temp.desc = "Write amplification factor";
  list.push_back(temp);

  temp.name = prefix + "fast.wear_leveling";
  temp.desc = "Wear-leveling factor";
  list.push_back(temp);

  temp.name = prefix + "fast.erase.count";
  temp.desc = "Total erase count";
  list.push_back(temp);

  for (uint32_t i = 0; i < chipEraseCount.size(); i++) {
    temp.name = prefix + "fast.erase.chip" + std::to_string(i);
    temp.desc =
        "Erase count of blocks with index % chips == " + std::to_string(i);
    list.push_back(temp);
  }

  // 아래 채널 통계는 lane = 채널(SuperblockSize = C)이라고 보고 센다
  for (uint32_t i = 0; i < chanErase.size(); i++) {
    temp.name = prefix + "fast.erase.channel" + std::to_string(i);
    temp.desc = "Erase count of physical blocks in channel " + std::to_string(i);
    list.push_back(temp);
  }

  for (uint32_t i = 0; i < chanWrite.size(); i++) {
    temp.name = prefix + "fast.write.channel" + std::to_string(i);
    temp.desc = "NAND write count (I/O units) in channel " + std::to_string(i);
    list.push_back(temp);
  }

  for (uint32_t i = 0; i < chipWrite.size(); i++) {
    temp.name = prefix + "fast.write.chip" + std::to_string(i);
    temp.desc = "NAND write count (I/O units) of blocks with index % chips == " +
                std::to_string(i);
    list.push_back(temp);
  }

  temp.name = prefix + "fast.buffer.flushes";
  temp.desc = "Total RW write buffer flush count";
  list.push_back(temp);

  temp.name = prefix + "fast.buffer.absorbed";
  temp.desc = "Total sectors overwritten in RW write buffer (not programmed)";
  list.push_back(temp);

  temp.name = prefix + "fast.buffer.write_ops";
  temp.desc = "NAND write requests issued by buffer flush (== flushes if each "
              "flush is one superpage)";
  list.push_back(temp);
}

void FAST::getStatValues(std::vector<double> &values) {
  values.push_back(stat.switchMerge);
  values.push_back(stat.partialMerge);
  values.push_back(stat.fullMerge);
  values.push_back(stat.rwVictims);
  values.push_back(stat.sectorCopies);
  values.push_back(stat.hostWritePages);
  values.push_back(stat.nandWritePages);
  values.push_back(stat.hostWritePages
                       ? (double)stat.nandWritePages / stat.hostWritePages
                       : 0.0);
  values.push_back(calculateWearLeveling());
  values.push_back(stat.eraseCount);

  for (auto &iter : chipEraseCount) {
    values.push_back(iter);
  }

  for (auto &iter : chanErase) {
    values.push_back(iter);
  }

  for (auto &iter : chanWrite) {
    values.push_back(iter);
  }

  for (auto &iter : chipWrite) {
    values.push_back(iter);
  }

  values.push_back(stat.bufferFlushes);
  values.push_back(stat.bufferAbsorbed);
  values.push_back(stat.bufferWriteOps);
}

void FAST::resetStatValues() {
  memset(&stat, 0, sizeof(stat));
  std::fill(chipEraseCount.begin(), chipEraseCount.end(), 0);
  std::fill(chipWrite.begin(), chipWrite.end(), 0);
  std::fill(chanWrite.begin(), chanWrite.end(), 0);
  std::fill(chanErase.begin(), chanErase.end(), 0);
}

}  // namespace FTL

}  // namespace SimpleSSD
