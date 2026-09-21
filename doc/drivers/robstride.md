# RobStride RS00 (`robstride,controller`)

RS00をClassic CANの29-bit extended frameで扱う型付きdriverである。generic motor APIへ
float位置やparameter readbackを詰めず、`include/drivers/motor/robstride.h`を使う。

## Wire contract

IDは `(type << 24) | (data16 << 8) | target`。hostからの`data16`下位byteは
master ID（既定`0xfd`）、motorからのbit15..8はsource motor IDである。Type 2の
position/velocity/torque/temperatureはBE16、Type 17/18のindexと値はLEである。

通常アプリが使うtypeはfeedback(2)、enable(3)、stop/clear(4)、parameter
read(17)、write(18)、fault raw(21)である。CAN ID変更、baudrate変更、flash保存、
active reportingはこのdriverの運転APIに含めない。

## Devicetree

```devicetree
robstride0: robstride-controller {
        compatible = "robstride,controller";
        #address-cells = <1>;
        #size-cells = <0>;
        can = <&xl2515>;
        master-id = <0xfd>;
        feedback-timeout-ms = <100>;
        command-timeout-ms = <100>;
        external-bus-management;

        shoulder: motor@1 {
                compatible = "robstride,rs00";
                reg = <1>;
        };
};
```

`external-bus-management`ではdriverはCANをstart/stopしない。共有bus ownerが
CAN開始後に`robstride_transport_start()`を呼ぶ。

## Safety and sequencing

`robstride_submit_position()`は有限値かつ±12.57 radだけを受理し、範囲外を黙って
clampしない。position commandは軸ごとのlatest-wins mailboxで、通常指令のCAN送信は
専用workerがcallback付き`K_NO_WAIT`で行う。command timeoutとoperation epochを過ぎた
指令は送らない。

`robstride_prepare_csp()`はstop、CAN timeout設定、run mode 5のreadback、`mech_pos`
read、現在位置への`loc_ref` seedまでを行い、enable frameは送らない。
`robstride_commit_enable()`が速度/電流上限を書いた後にenableする。parameter待ちは
motorごとに1件で有限timeout、STOPは待機を`-ECANCELED`で起こす。同じindexの遅延応答を
wire上で完全識別するtransaction IDはないため、timeout後は手順全体を中断する。

Type 2 feedbackはraw 16-bit位置を±12.57 radへ復号し、wrap turnを追跡する。長いgapで
continuous flagはラッチして落ち、次の1 frameでは復帰しない。`mech_pos`との整合を
確認した`robstride_rebase_position_from_mech_pos()`だけが再基準化する。

## Verification

`tests/drivers/motor/robstride`はgolden frame、filter/decode、連続性、型付きreadback、
CSP順序、epoch stop、非有限/範囲外拒否をfake CANで試験する。これは実機firmwareの
0x7028 timeout挙動や停止時間を保証しない。
