# FAST-LIO AVIA ROS2 Porting Notes

## Overview
FAST-LIO를 ROS2로 포팅하면서 AVIA SDK1 드라이버(PointCloud2 출력)에 맞게 수정한 사항 정리.
레퍼런스: https://github.com/Ericsii/FAST_LIO_ROS2 (CustomMsg 기반)

## 핵심 코드 수정

### 1. preprocess.cpp — AVIA PointCloud2 핸들러
SDK1 드라이버는 CustomMsg 대신 PointCloud2를 출력하므로 별도 핸들러 작성.

**포인트 포맷**: `x(0), y(4), z(8), intensity(12), tag(16), line(17), t(18)`, point_step=22

**레퍼런스 대비 주의사항**:
- `point_filter_num` 적용하지 않음 — AVIA handler는 모든 유효 포인트 사용 (레퍼런스 동일)
- tag 필터: `(tag & 0x30) == 0x00 || 0x10`만 통과 (multi-return echo 제거)
- scan line 필터: `line >= N_SCANS` 제거
- 연속 중복 포인트 필터: `abs(x_i - x_{i-1}) > 1e-7` 등
- `given_offset_time = true` 고정
- 타임스탬프: `t` 필드(UINT32, 나노초 오프셋) × `time_unit_scale(1e-6)` → 밀리초

### 2. laserMapping.cpp — IMU 큐 크기
```cpp
// 원래: 10 (50ms 분량, 200Hz IMU에서 드롭 발생)
// 수정: 2000 (10초 분량)
sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 2000, imu_cbk);
```

## Config (avia.yaml)

```yaml
filter_size_surf: 0.3       # 전체 포인트 사용하므로 적절한 다운샘플링
filter_size_map: 0.3        # surf와 동일
acc_cov: 0.1                # AVIA IMU는 g단위 출력, 0.01은 너무 낙관적
gyr_cov: 0.01               # 원본 유지
fov_degree: 70.4            # AVIA 실제 FOV (원본 90은 범위 초과)
det_range: 150.0            # 원본 50에서 확장
extrinsic_est_en: false     # factory calibration 사용
timestamp_unit: 3           # NS (필수)
dense_publish_en: true      # dense point cloud 출력
```

## IMU 초기화 주의사항
- FAST-LIO는 첫 10 LiDAR 프레임(~1초) 동안 IMU 평균으로 gravity/gyro bias 추정
- **반드시 센서 정지 상태에서 시작**해야 함
- bag 재생 시 움직이는 구간이 앞에 있으면 `--start-offset`으로 정지 구간부터 시작

## 알려진 한계
- AVIA SDK1 드라이버의 zero point (~47%가 (0,0,0)) → `blind=1.0`으로 필터됨
- 반드시 Release 빌드 (`-DCMAKE_BUILD_TYPE=Release`)
