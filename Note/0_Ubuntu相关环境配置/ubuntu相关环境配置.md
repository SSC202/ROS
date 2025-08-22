# Ubuntu 相关环境配置(主机)

> 本篇介绍 Ubuntu 相关的环境配置。
>
> 网络上关于 Linux 的基础教程很多，这里不再赘述。

## 1. Ubuntu 系统的安装和卸载

### 安装 Ubuntu 系统

1. 制作启动盘。使用 U 盘和常见的镜像烧录软件即可。

   Ubuntu 镜像可以在清华大学开源镜像站找到：[网址](https://mirrors.tuna.tsinghua.edu.cn/#)

   > 下载镜像请注意**系统架构**(amd64/arm64/...)和**安装类型**(服务器/桌面)，通常主机是amd64架构(Orangepi是arm64)。一般安装桌面类型镜像。

2. 重启，重启后按住对应的按键进入 BIOS。

   - 关闭安全启动模式。
   - 将启动盘作为最高优先级的启动项。
   - 退出继续重启。

3. 此时进入 `grub` 界面，可以看到很多开机选项，选择`Try or Install Ubuntu`。

4. 按照提示进行 Ubuntu 安装程序即可。

   - 可以选择不联网安装，这样更快；

   - 可以选择仅安装基本组件，同时**不要选择安装第三方图形驱动**(后期调整驱动版本将会十分费劲)。

   - 对于单系统，可以选择磁盘格式化后安装；对于双系统，选择自定义安装，进行分盘：

     > - `efi` 引导分区： 512 MB，其余默认。
     > - `swap` 交换分区：通常选择大于内存大小的值，其余默认。
     > - `brtf` 分区：其余空间，选择 Logical 分区，挂载(mount point)根目录 `/`。
     > - 最后选择引导分区(分区界面下面的`bootloader point`)，选择新分出来 EFI 的磁盘号。(***注意不要错选了 Windows 的 EFI分区***)

5. Ubuntu 安装完成，重启电脑，此时拔出启动盘。

### 安装 Ubuntu 后立刻做的事情

#### 换源

这是为了加快 `apt install` 的速度，不然就必须科学上网。

直接使用 **小鱼一键换源** 即可。

```shell
$ wget http://fishros.com/install -O fishros && . fishros
```

#### 时间同步

Windows 与 Linux 缺省看待系统硬件时间的方式是不一样的：

Linux 时钟分为系统时钟(System Clock)和硬件(RTC)时钟。Linux 把硬件时间当作 UTC,系统时间是 UTC 时间经过换算得来的。比如说北京时间是 GMT+8,则系统中显示时间是硬件时间+8。 Windows 把系统硬件(RTC)时间当作本地时间(local time)，即系统时间跟 BIOS 中显示的时间(RTC)是一样的。

通过以下指令同步时间：

```shell
$ sudo apt-get install ntpdate					# 在Ubuntu下更新本地时间
$ sudo ntpdate time.windows.com
$ sudo hwclock --localtime --systohc			# 将本地时间更新到硬件上
```

### 卸载 Ubuntu

> 环境乱了？卸载 Ubuntu 重装，一劳永逸。

1. 把开机启动项设为默认 Windows 启动。

2. 使用磁盘管理工具删除 Ubuntu 系统分区。

3. 删除 EFI 启动项

   - 方法1：使用 EasyUEFI 软件 —— 管理 EFI 启动项 —— 删除ubuntu启动项；
   - 方法2：为 windows EFI 系统分区分配盘符
     - `win + r` 打开运行，输入 `diskpart`；
     - 输入 `list disk` 寻找 Windowd EFI 系统分区(类型为系统)；
     - 输入 `select disk 1` ，接着输入 `list partition` 查看具体分区列表，根据容量找到 Windows EFI 系统分区。
     - 为 Windows 的 EFI 系统分区分配盘符 `assign letter = e` ，这里 e 为盘符(不区分大小写)，**不要和已有的盘符重复**，分配完成后不要关闭此窗口，进入资源管理器可看到 E 盘。
     - 直接打开 E 盘会发现权限不够，打不开。 这里我们要运用一个小技巧，先用管理员权限打开记事本，然后通过记事本菜单栏里的【打开】来访问，可以看到 EFI 文件夹，进入找到 ubuntu 文件夹，删除即可。
     - 输入 `remove letter=e`，关闭窗口退出，完成。

   

