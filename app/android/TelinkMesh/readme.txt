
2020.12.03 update(V2.6.2)
1. Demo更新
    1.1 修复Mesh OTA过程中杀后台程序再重新进入app后没有更新OTA进度信息的问题
        Fix OTA progress not updating error after kill process when Mesh OTA running

============================================================================================================


2020.08.21 update(V2.6.1)
1. Demo更新
    1.1 OTA 升级判断条件由mac地址相同改为mesh短地址相同
        update OTA permission from mac-address equality to mesh-address equality

============================================================================================================


2020.06.19 update(V2.6.0)
1. Demo更新
    1.1 更新二维码扫描前的权限申请流程；
        update camera permission request flow;
    1.2 更新 button和icon等UI。
        update button and icon style
============================================================================================================

2019.10.29 update(V2.5)
1. Demo更新
    1.1 移除部分UI系统上的兼容性提示: detected problems with api compatibility
2. Lib更新
    2.1 移除加密相关的so库， 采用java实现的方式替换

============================================================================================================

2019.01.05 update(V2.4)
1. Lib更新
    1.1 OTA流程中的第一个包和第二个包添加300ms间隔
    add 300ms spacing time between 1st and 2nd pkt during OTA;
2. Demo 更新
    2.1 MeshScan 过程增加retry： 在配置设备id时， 如果未返回， 则重新获取该设备的device id;
    MeshScan流程可参考DeviceMeshScanningActivity类和Telink_Mesh_Scan_Flow文档;
    通过MeshScan添加的设备会缺失productUUID字段, 可能会影响MeshOTA功能(配置OTA模式时会添加productUUID信息), 可以通过填写与设备一致的productUUID进行升级;

    Add retry when scan devices by mesh: get target device id if no rsp after setting device id;
    DeviceMeshScanningActivity class and Telink_Mesh_Scan_Flow doc are for reference as mesh scan flow;
    Devices scanned by mesh are lack of productUUID, so mesh ota function will be effected, but with the same productUUID as device's updating can also proceed

============================================================================================================

2018.04.28 update(V2.3)

1. Lib更新
    1.1 扫描接口添加LeScanParameters.setScanTypeFilter(int type)， 用于过滤扫描的设备类型(仅上报该类型);
    1.2 Manufacture 中的默认vendorId 由 0x1102 改为 0x0211, 并在对应调用位置做大小端修改；

2. Demo更新
    2.1 MeshOTA 流程修改: 新增设置OTA模式指令;  新增静默OTA功能:  参考MeshOTAService 类实现相关功能;
    2.2 单灯OTA流程修改: OTA完成后增加回读动作， 读取设备的版本号， 判断是否与对应firmware版本一致， 来确定OTA成功；
    2.3 demo中降低online status 设备控制权限: 对于未加灯设备， 设定为只可开关灯
    2.4 demo 中新增Mesh加灯功能， 对应DeviceMeshScanningActivity类， 可通过选择主页标题栏MeshScan再点击'+'进入;

============================================================================================================

2017.12.29 update(V2.2)
demo 修改：
1. mesh 分享功能添加, 在mesh setting页面点share mesh可以分享/扫描二维码；
2. OTA 发包时， 少于20字节的会在content部分补0xFF, 与iOS统一。   OTA pkt 结构： index[2], content[16], crc[2]

============================================================================================================


2017.11.09 update (V2.1)
sync
release version

============================================================================================================

2017.10.30 update (V2.0.2)
1. 采样类修改， 指令最后时间lastCmdTime加入delay的延时，保证指令发送正常， 目的在于 修复在重复发送非采样指令时，期间插入指令导致设备相应不正常问题。

2. 小米6手机 兼容性问题： 加灯过程中会出现 闪灯成功，app却加灯失败， 通过抓包分析，小米手机在发送write_response指令时，把ack当作response，导致后面的包乱序。
    解决方法， 在reset的4条指令中间添加一定时间(默认200ms)的delay；

3. ErrorReport 的埋点信息中添加deviceId(meshAddress).

4. 修改7.0及以上系统Ble扫描机制， 以修复华为部分手机兼容性问题：【在7.0及以上系统中扫描时发起连接则会出现连接不上(无法建立连接或者读取ATT表失败)】。
    修改点：
    1.1 扫描停止动作由延时处理改为即时处理;
    1.2 扫描开启动作会判断上一次扫描成功开启时间，少于 【6s】 时会delay 时间差值，以保证【在反复开启扫描不会出现扫描不到】(@quote 4.10 update)的情况;

5. Alarm解析器类GetAlarmNotificationParser中 parse() 方法修改: 修复解析错误的问题;


============================================================================================================

2017.10.18 update (V2.0.1)
1.添加ErrorReport相关机制

============================================================================================================

2017.08.09 update (V2.0.0)
1. OTA过程中获取设备状态时，重试3次仍获取不到则继续OTA操作；
2. OTA版本对比问题修复;
3. 清除废弃的内部文件;
============================================================================================================

5.22 update
1.修复 发送指令后更改系统时间至更早，会导致指令发送不出去 问题：
    在AdvanceStrategy 采样方法中添加 if (interval < 0) interval = 0  即当前时间取值小于之前指令发送时间，则置为零。
2.采样策略中移除默认320ms delay，保证指令的准时发送。
3.Demo修改：
    3.1 开关灯时，以online_status状态为准；
    3.2 调节亮度时，范围改为5-100；

============================================================================================================

5.19 update
针对firmware在Android N（7.0）版本有较大兼容性问题,Android这边主要是添加UI提醒
1.LightController类修改：
    1.1 添加 N 版本判断和连接重试方法，以及相应变量， 具体可以参考命名和相关注释 // android N check
    1.2 在内部类LightEvent中添加 常量 CONNECT_FAILURE_N 用于标识错误类型
2.LightAdapter类修改：
    1.1 添加常量 STATUS_ERROR_N，标识N错误，调用setStatus(STATUS_ERROR_N)，可讲错误事件分发； 可参考DeviceScanningActivity中使用；

============================================================================================================

4.26 update
1.自动连接参数中，添加自动连接的MAC，设置可连接指定mac设备；

============================================================================================================

4.19 update
1.LeBluetooth类中添加对Android M（6.0）支持，具体为：在M以上版本会判断Location是否开启,未开启会回调onScanFail(int errorCode),
    前端在调用扫描接口时，建议添加监听 MeshEvent.ERROR,并判断errorCode,若是Location问题,可以通过Intent跳转设置页面打开Location开关.

2.采样策略类优化 AdvanceStrategy.DefaultAdvanceStrategy：除了第一条指令和特别声明的立即发送的指令外，其它控制指令都会默认delay 320ms；
    同时对特定指令采样 默认320ms的采样间隔。

============================================================================================================

4.10 update
1. 经过反复测试可以确认在Android 7.0版本会出现如下情况：30s内有5次开关蓝牙扫描动作，第6次会开启动作会不生效。这个是Android蓝牙底层控制的，上层无法通过接口参数更改。
据此在SDK的LightAdapter类中添加扫描延时机制：
    1.1 一个扫描周期的最少时间为10s;
    1.2 10s蓝牙关闭操作会延时到10s执行，期间有重新开启扫描则取消Timer;
    1.3 低版本设备保留之前的扫描机制.
其它更新：
2.删除LightService类中的sendCommand 接口，仅保留一个sendCommandNoResponse;
3.在自动连接模式下，添加扫描设备队列，连接重试机制，重试一次，失败后会从队列中获取;【deprecated】
4.删除不用的delete接口;
5.LightAdapter类中去掉处理扫描结果的子线程;
