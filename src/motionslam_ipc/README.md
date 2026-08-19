# motionslam_ipc

`motionslam_ipc` 是不依赖 ROS 消息序列化的 Linux POSIX SHM 点云数据面。协议为
固定 slot 的 SPSC 环：生产者永不等待消费者，写满后覆盖最旧帧，消费者通常只读
最新完整帧。

## SCAN reader 集成

```cpp
#include <motionslam_ipc/shm_cloud_ring.hpp>

auto ring = motionslam_ipc::ShmCloudRingReader::Open(
    "/motionslam_lio_cloud_world");
motionslam_ipc::CloudFrame frame;
if (ring.ReadLatest(frame) == motionslam_ipc::ReadResult::kSuccess) {
  // frame.points 是 world 坐标系下的紧凑 XYZI(float32)。
  // generation 变化表示生产者重启；session_id 变化表示数据会话变化。
  // tile_id 由生产者参数指定，当前 Super-LIO 默认值为 0。
}
```

读端必须把 `magic`、schema 和 ABI 校验失败视为不可兼容，不能猜测布局。
`ReadLatest()` 返回 `kOverwritten` 时直接重试即可；这不是数据损坏。

Super-LIO 参数：

- `lio.output.cloud_world_topic.enabled`：默认 `true`；开启时才创建
  `/lio/cloud_world` publisher，并执行 PointCloud2 转换与 DDS 发布
- `lio.output.shm_cloud.enabled`：默认 `true`
- `lio.output.shm_cloud.name`：默认 `/motionslam_lio_cloud_world`
- `lio.output.shm_cloud.slot_count`：默认 `4`
- `lio.output.shm_cloud.max_points`：默认 `200000`
- `lio.output.shm_cloud.session_id`：默认 `0`，由 writer 自动生成
- `lio.output.shm_cloud.tile_id`：默认 `0`

SHM 写入直接从 PCL `PointXYZI` 生成紧凑 XYZI，不经过 ROS 消息。只有
`lio.output.cloud_world_topic.enabled=true` 时才保留 `/lio/cloud_world`
调试/兼容 topic；关闭后不会创建 publisher，也不会执行 `pcl::toROSMsg`。
`hybrid_nav_stack.launch.py` 生产路径显式覆盖为 `false`，历史 pure Nav 配置显式
保持 `true`。若 SHM 初始化失败且 topic 同时关闭，则该进程不会输出 world 点云。

## eventfd

`CreateEventFd()`、writer 的 `notification_fd` 和 reader 的
`WaitForNotification()` 提供可选通知。eventfd 没有可按名字重新打开的全局端点；
独立启动的进程必须通过父进程继承或 Unix socket `SCM_RIGHTS` 传递同一个 fd。
未建立 fd 传递通道时，SCAN 应按自身更新周期调用 `ReadLatest()`，不要尝试用
SHM 名字推导 eventfd。

SHM 对象不会在 writer 析构时自动 unlink，便于生产者重启时递增 generation。
部署清理可调用 `UnlinkShmCloudRing()` 或使用 `shm_unlink`。
