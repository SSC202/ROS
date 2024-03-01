# ROS2 A1_MicroROS

## 1. MicroROS 简介

## 2. MicroROS STM32配置

### 上位机配置

1. 安装ROS-Humble
1. 下载ROS-Agent

```sh
sudo apt-get install -y build-essential
mkdir -p microros_ws/src
cd microros_ws/src
git clone http://github.fishros.org/https://github.com/micro-ROS/micro-ROS-Agent.git -b humble
git clone http://github.fishros.org/https://github.com/micro-ROS/micro_ros_msgs.git -b humble
cd microros_ws
colcon build
```

2. 运行Agent

```sh
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyUSB0 -v
```

### 下位机配置

参考：

https://github.com/TonyKerman/WTR_micro_ros_stm32cubemx_utils

https://github.com/micro-ROS/micro_ros_stm32cubemx_utils

1. 建立STM32CubeMX工程

- 使用 FreeRTOS CMSIS_V2 封装；
- 使用串口，打开串口全局中断；
- 打开串口DMA的 RX 和 TX 通道，其中 RX 通道设置为 Circular；
- 将 FreeRTOS 的 default task 堆栈大小设置为 3000；
- 使用 Makefile 方式生成工程。

2. 修改`Makefile`文件

在`build the application`之前添加以下代码：

```makefile
#######################################
# micro-ROS addons
#######################################
LDFLAGS += WTR_micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
C_INCLUDES += -IWTR_micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include

# Add micro-ROS utils
C_SOURCES += WTR_micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
C_SOURCES += WTR_micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
C_SOURCES += WTR_micro_ros_stm32cubemx_utils/extra_sources/microros_time.c

# Set here the custom transport implementation
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c

print_cflags:
	@echo $(CFLAGS)
```

3. 克隆官方库到工程中

```sh
git clone -b humble https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
```

4. 使用`Dockerfile`构建`microros`静态库生成工具。

https://github.com/TonyKerman/WTR_micro_ros_stm32cubemx_utils/tree/humble/micro-ros-lib-builder

```sh
#在micro-ros-lib-builder下
docker build -t micro-ros-lib-build-humble .
```

构建成功后，你可以在本地镜像列表中看到`micro-ros-lib-build-humble`镜像，下次不需要再次构建。

5. 生成静态库

在工作空间中执行：

```sh
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library micro-ros-lib-build-humble
```

6. 编写文件

```c
// 包含以下头文件
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

// 包含消息类型
#include <std_msgs/msg/int32.h>

// 函数定义
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// DefaultTask 任务
void StartDefaultTask(void *argument)
{
    // micro-ROS configuration

    rmw_uros_set_custom_transport(
            true,
            (void *) &huart3,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
        printf("Error on default allocators (line %d)\n", __LINE__);
    }

    // micro-ROS app

    rcl_publisher_t publisher;
    std_msgs__msg__Int32 msg;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    allocator = rcl_get_default_allocator();

    //create init_options
    rclc_support_init(&support, 0, NULL, &allocator);

    // create node
    rclc_node_init_default(&node, "cubemx_node", "", &support);

    // create publisher
    rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "cubemx_publisher");

    msg.data = 0;

    for(;;)
    {
        rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
        if (ret != RCL_RET_OK)
        {
            printf("Error publishing (line %d)\n", __LINE__);
        }

        msg.data++;
        osDelay(10);
    }
    /* USER CODE END 5 */
}
```

7. `CMakeLists.txt`修改

```cmake
# 使用硬件浮点选项
# Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# 头文件包含
include_directories(
...
micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include)

# 库文件包含
file(GLOB_RECURSE SOURCES ... "micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c" "micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c" "micro_ros_stm32cubemx_utils/extra_sources/microros_time.c" "micro_ros_stm32cubemx_utils/extra_sources/microros_transports/dma_transport.c")

# 包含静态连接库
# 构建绝对路径
set(LIBMICROROS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros")
# 添加库文件搜索路径
link_directories(${LIBMICROROS_PATH})
# 链接库，使用库的实际名称
target_link_libraries(${PROJECT_NAME}.elf PRIVATE ${LIBMICROROS_PATH}/libmicroros.a)
```

