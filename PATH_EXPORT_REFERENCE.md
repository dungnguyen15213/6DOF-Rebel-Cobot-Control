# CRI Path Export Reference

`PathExecutor` stores each waypoint as six joint angles: `A1` through `A6`, in degrees. It can export:

| Format | Content |
|---|---|
| CRI CSV | Ordered `CMD Move Joint` payloads |
| Joint CSV | Ordered six-axis targets with controller velocity |
| JSON | CRI joint target fields and extension axes |

The controller is responsible for target timing and motion profiles.