## DeepRadar: Deep Learning for TI mmWave radars.

This projects makes an attempt to get insights from nearly-raw radar data,
just lightly processed with FFT. [Early experiments](https://ivankrasin.com/2018/04/09/deep-learning-for-radars-a-quick-glance/) show that it's trivial to achieve 90+% accuracy for simple classifiers. The project goal is to establish boundaries of what's possible with this approach.

### To build the firmware:

1. Get [TI mmWave SDK](http://www.ti.com/tool/MMWAVE-SDK) and install it locally.

2. Check out this repository into ```${MMWAVE_SDK_INSTALL_PATH}/packages/ti/demo/xwr14xx/DeepRadar```.

3. Copy ```${MMWAVE_SDK_INSTALL_PATH}/packages/ti/demo/xwr14xx/mmw/mmw.cfg``` into ```${MMWAVE_SDK_INSTALL_PATH}/packages/ti/demo/xwr14xx/DeepRadar/mmw.cfg```. This file is not released under open source license by TI and can't be included into this repository.

4. Set environment variables via ```${MMWAVE_SDK_INSTALL_PATH}/packages/scripts/unix/setenv.sh``` as described in [TI mmWave SDK User Guide](http://software-dl.ti.com/ra-processors/esd/MMWAVE-SDK/latest/exports/mmwave_sdk_user_guide.pdf).

5. Build the mss binary:

```
DeepRadar$ make all
Configuring RTSC packages...
<...>
******************************************************************************
Built the Millimeter Wave OUT and BIN Formats
******************************************************************************
```

6. Flash rss and mss binaries into your [TI IWR1443BOOST board](http://www.ti.com/tool/IWR1443BOOST). See "How to flash an image onto xWR14xx/xWR16xx EVM" in the [TI mmWave SDK User Guide](http://software-dl.ti.com/ra-processors/esd/MMWAVE-SDK/latest/exports/mmwave_sdk_user_guide.pdf).

Note: flashing might not properly work on Linux. Use a Windows machine for this operation as a workaround.

