# static_executor
Library that adds alternative(s) to the default executor in ROS2
## How to use
1. Add static_executor to your CMakeLists.txt and package.xml as a dependency.

2. Add
```
#include "static_executor/executors/static_single_threaded_executor.hpp"
```
at the top of your code.

3. You can now replace
```
rclcpp::executors::SingleThreadedExecutor exec;
```
with
```
rclcpp::executors::StaticSingleThreadedExecutor exec;
```
in your source code. 

No other changes are required.

## Depends on 
```
rclcpp
```
