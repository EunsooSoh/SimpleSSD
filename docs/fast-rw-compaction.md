# FAST RW 로그 간 이동 GC

## 사용 설정

현재 `nexus5.cfg`는 2026-11-22의 5세트 실험과 같은 Greedy victim 선택과
로그 간 이동 GC를 사용한다: `FASTRWGCPolicy=1`, `FASTRWCompaction=1`,
`FASTCommonWarmup=1`. 실험은 정책 1을 유지하고 이동 OFF/ON을 비교했다.
아래 정책 2는 선택 가능한 구현이며 해당 5세트 실험의 사용 정책은 아니다.

## 비용 기준 victim 선택

`FASTRWGCPolicy=2`는 모든 회수 가능한 RW 로그를 비교함. 각 후보에서
`full merge 복사량 / 기존에 사용한 쓰기 공간`과, 이동이 가능하면
`유효 sector 수 / (기존 사용 공간 - 목적지에 쓰는 공간)`을 비교함.
후보별 최저 비용을 구한 뒤 그중 가장 작은 후보를 선택함.
`FASTRWCompaction=0`이면 full merge 비용만 비교함.

예상 full merge 복사량은 관련된 서로 다른 LBN의 최신 NAND 데이터 전체이며
DRAM 버퍼에만 있는 데이터는 제외함. 같은 LBN의 계산은 후보 탐색 1회 안에서
재사용함. 선택 평가는 매핑·유효 상태·쓰기 위치를 변경하지 않음.
동일 점수면 오래된 로그를 고르고, 이동/full merge 동점이면 full merge함.
분수는 교차 곱 오버플로 없이 비교함. 회수 공간이 0인 후보는 후순위임.

버퍼 ON에서는 열린 로그를 후보에서 제외함. 버퍼 OFF에서 공간 부족으로
실행될 때는 현재 요청 lane에서 실제로 회수되는 공간을 기준으로 함.
반복 이동 제한과 NAND 기록 완료 후 원본 erase 순서는 유지함.
`FASTRWGCPolicy=1`은 기존 Greedy를 유지하므로 기준선 비교에 사용할 수 있음.

비용 기준 정책을 별도로 비교할 때의 설정: `FASTRWGCPolicy=2`, `FASTRWCompaction=1`,
`FASTCommonWarmup=1`. 기준선은 `FASTRWGCPolicy=1`,
`FASTRWCompaction=0`, `FASTCommonWarmup=1`임.

추가 통계 `rw.cost_selections`는 비용 기준 선택 횟수,
`rw.cost_different_from_greedy`는 같은 시점의 단순 Greedy와 다른 로그를
선택한 횟수임. 이 방식은 RW 로그를 선택하는 안이며 전체 LBN을 대상으로
merge 순서를 정하는 별도의 안은 구현하지 않음.

`[ftl] FASTRWCompaction = 1`이면 설정한 정책으로 고른 RW 로그의 최신 sector만
새 RW 로그로 옮기는 방법을 먼저 평가함. 기존 데이터 블록은 합치지 않음.
기본값은 0이며, 로컬 `nexus5.cfg`에서는 1로 활성화함.

## 이동 조건

- 선택한 로그에 유효 데이터와 이미 사용한 무효 공간이 모두 있어야 함.
- `이동 sector 수 / 다시 확보되는 쓰기 공간`이
  `full merge 복사 sector 수 / full merge로 다시 확보되는 쓰기 공간`보다
  작아야 함. 동률이면 기존 full merge를 수행함.
- 버퍼 OFF에서는 현재 쓰기에 필요한 lane의 공간을 비교하며, 이동 후에도
  논리 lane을 유지함. 버퍼 ON에서는 전체 lane 공간을 비교하고 고르게 배치함.
- 공간은 로그 전체 용량이 아니라 기존 write pointer까지 이미 사용한 칸을
  기준으로 계산함. 이동 시 목적지에 쓰는 칸은 회수 이득에서 차감함.
- full merge 예상 복사량은 관련 LBN의 최신 NAND sector를 조회해 합산함.
  DRAM 쓰기 버퍼에만 있는 sector는 제외함.
- 같은 최신 버전은 한 번만 로그 간 이동함. 다시 이동해야 하면 full merge함.
  호스트가 덮어쓴 새 버전은 다시 이동할 수 있음.
- 유효 데이터가 없는 로그는 복사 없이 바로 erase함.

## 공간과 실행 순서

기존 OP 여유 블록 중 1개를 임시 목적지로 사용함. 이동 중에는 원본과 목적지가
동시에 존재하지만, 원본 erase 후 목적지가 같은 RW 로그 자리를 대체하므로
상시 RW 로그 수와 전체 물리 용량은 증가하지 않음. 목적지의 남은 공간은
현재 요청과 이후 쓰기가 재사용함. 영구적으로 추가 로그를 예약하지 않음.

이전 NAND 작업 완료 → 원본 읽기와 목적지 쓰기 완료 → 원본 erase 순서임.
로그의 무효 칸은 erase 전에는 재사용하지 않음. 실패/정전 복구는 기존
시뮬레이터와 마찬가지로 별도 모델링하지 않음.

## 통계와 비교

`FASTCommonWarmup = 1`을 두 실험 모두에 설정하면 초기화 중에는 Greedy와
이동 OFF를 공통으로 사용하고, 완료 후 요청한 실행 정책을 복원함.
기본값 0은 기존 동작을 유지함. 공통 초기화 종료 시 배치·쓰기 포인터·유효
데이터·프리 블록 순서·버퍼·매핑을 포함한 64비트 상태 지문을 stderr의
`FAST_COMMON_WARMUP state=...`에 기록함. 비교 실행의 지문이 같은지 확인해야 함.

- `ftl.fast.rw.compactions`: 로그 간 이동으로 회수한 RW 로그 수
- `ftl.fast.rw.compaction_copies`: 이동한 sector 수
- `ftl.fast.rw.compaction_fallbacks`: 이동 조건을 만족하지 못해 full merge한 수
- 이동 복사는 기존 `merge.sector_copies`, NAND 쓰기와 WAF에도 포함됨.

같은 FIFO/Greedy 정책, RW 로그 개수, 물리 용량, 버퍼 설정, 초기화와 입력을
유지하고 `FASTRWCompaction`만 0/1로 비교해야 함. 선택된 victim만 평가하는
방식임. 정책 0/1은 먼저 고른 victim만 평가하고, 정책 2는 후보 전체의 비용을 비교함.
버퍼 ON에서 회수 가능한 닫힌 로그가 하나뿐이면 victim 정책 간 차이는
작을 수 있음. 테스트 통과는 정합성 검증이며 성능 개선을 보장하지 않음.

후보의 복사량을 조사하는 C++ 연산 시간은 별도 NAND/FTL 지연으로 추가하지
않음. 따라서 시뮬레이션 지연과 실제 호스트 실행 시간을 구분해서 평가해야 함.

## GC 대기 시간과 5세트 실험 (2026-11-22)

`fast.gc.{all,sw,rw_merge,rw_compaction}.{events,wait_ps,max_wait_ps}`는
초기화 이후 동기식 회수 함수 호출 횟수, 호출 구간의 시뮬레이션 시간 합계와
최대를 출력한다. `sw`는 `mergeSW()`, `rw_merge`는 이동을 하지 않은
`mergeRWVictim()`(빈 로그 회수 포함), `rw_compaction`은 이동 성공 경로이다.
평균은 `wait_ps / events`이며 호출이 없으면 평균은 정의하지 않는다.

측정 구간은 함수 진입 시 `batch.doneTick`부터 반환 시까지이다. 기존 실행 순서와
배치를 유지하며 계측 때문에 추가 flush나 PAL 요청을 넣지 않는다. 따라서 GC
호출 안에서 기존 NAND 작업이나 함께 대기 중인 호스트 쓰기를 처리한 시간이
포함될 수 있다. 이 값은 **GC 회수 호출 구간 대기 시간**이며, GC만의 NAND
서비스 시간이나 GC가 각 호스트 요청에 추가한 인과적 지연과 같지 않다. 여러
요청이 병렬 처리되므로 합계는 전체 실행 시간으로 해석하지 않는다.

`scripts/run_gc_five_sets.py`(상위 standalone 저장소)는 RW 로그 8/4/16개,
버퍼 OFF, 초기 채움 70%의 다섯 조건을 실행한다. 기준은 RW 로그 8개, 버퍼 ON,
초기 채움 90%이다. 모든 조건에서 Greedy victim을 유지하고 이동 OFF/ON만
비교하며 각 쌍의 공통 초기 상태 지문을 대조한다. Booting을 제외한 30개 입력의
전체 요청을 각각 실행한다. 동일 내용의 MusicFaceBook 두 파일은 각각 표시한다.

`LatencyLogFile`의 요청 ID를 입력 순서의 읽기/쓰기에 연결하고 주소·길이·완료
수를 검증한다. 전체/읽기/쓰기 평균·p99·최대 지연을 모두 보존하며 p99는
nearest-rank(`ceil(0.99*N)`)이다. 복사량은 기존 `merge.sector_copies`의
FTL I/O unit 수이다. 현재 8채널·Multi-plane ON·Plane 2개 설정에서는
superPageSize=256KiB, ioUnitInPage=8이므로 1 I/O unit은 32KiB이며,
16KiB 물리 페이지 2개에 해당한다. 원본 카운터와 물리 페이지 환산값을 함께
출력한다. 기존 통계 및 요청별 지연 로그와 같은지 기준 조건을
재검증한다. 개별 실행 통계와 비교표는 `output/gc_compaction/five_sets_20261122/`
아래에 저장한다. GC 대기 통계가 없던 이전 실행을 새 대기 통계로 재사용하지
않으며, 새로 실행한 유효한 결과만 재개 시 검증 후 재사용한다.

## 정합성 테스트

기존 `test_fast`에 21개 집중 사례와 FIFO/Greedy × 버퍼 OFF/ON × lane 1/4 ×
압축 OFF/ON의 16개 조합 검사를 포함함. 로그 교체, 빈 로그 회수, 공간/비용
조건에 따른 full merge 복귀, 최신 매핑, 반복 이동 제한 및 호스트 덮어쓰기 후
새 버전의 이동 가능 여부를 검사함.
