/**
 *   @file  main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** @mainpage Millimeter Wave (mmw) Demo for XWR14XX
 *
 *  @section intro_sec Introduction
 *
 *  @image html toplevel.png
 *
 *  The millimeter wave demo shows some of the capabilities of the XWR14xx SoC
 *  using the drivers in the mmWave SDK (Software Development Kit).
 *  It allows user to specify the chirping profile and displays the detected
 *  objects and other information in real-time.
 *
 *  Following is a high level description of the features of this demo:
 *  - Be able to specify desired chirping profile through command line interface (CLI)
 *    on a UART port or through the TI Gallery App - **mmWave Demo Visualizer** -
 *    that allows user to provide a variety of profile configurations via the UART input port
 *    and displays the streamed detected output from another UART port in real-time,
 *    as seen in picture above.
 *  - Some sample profile configurations have been provided in the demo directory that can be
 *    used with CLI directly or via **mmWave Demo Visualizer**:
 *    @verbatim
        mmw/profiles/profile_2d.cfg
        mmw/profiles/profile_3d.cfg
        mmw/profiles/profile_heat_map.cfg
      @endverbatim
 *  - Do 1D, 2D, CFAR, Azimuth and Elevation processing and stream out velocity
 *    and three spatial coordinates (x,y,z) of the detected objects in real-time.
 *    The demo can also be configured to do 2D only detection (velocity and x,y coordinates).
 *  - Various display options besides object detection like azimuth heat map and
 *    Doppler-range heat map.
 *  - Illustrates how to configure the various hardware entities (Hardware accelerator (HWA),
 *    EDMA, UART) in the AR SoC using the driver software.
 *
 *  @section limit Limitations
 *  - Because of UART speed limit (< 1 Mbps), the frame time is more restrictive.
 *    For example, for the azimuth and Doppler heat maps for 256 FFT range and
 *    16 point FFT Doppler, it takes about 200 ms to transmit.
 *  - Present implementation in this demo cannot resolve objects at the same range and velocity
 *    but at different azimuth and/or elevation. The algorithms may be improved in future
 *    versions to solve this.
 *  - Code will give an error if the requested memory in L3 RAM exceeds its size (@ref SOC_XWR14XX_MSS_L3RAM_SIZE) due to
 *    particular combination  of CLI configuration parameters.
 *  - For most boards, a range bias of few centimeters has been observed. User can estimate
 *    the range bias on their board and correct using the calibration procedure
 *    described in @ref calibration.
 *  - For the scheme @ref DATA_PATH_CHAIN_COMBINED_LOGMAG, the azimuth static
 *    heat map (Doppler 0 range bins) is computed on the 1D FFT output instead
 *    of the required 2D FFT output because of which the heat map will get affected
 *    by motion in the scene. This will be corrected in a future release.
 *
 *  @section tasks Software Tasks
 *    The demo consists of the following (SYSBIOS) tasks:
 *    - @ref MmwDemo_initTask. This task is created/launched by @ref main and is a
 *      one-time active task that performs the following sequence:
 *      -# Initializes drivers (\<driver\>_init).
 *      -# Initializes the MMWave module (MMWave_init)
 *      -# Creates/launches following three tasks (the @ref CLI_task is launched indirectly by
 *          calling @ref CLI_open).
 *    - @ref CLI_task. This command line interface task provides a simplified 'shell' interface
 *      which allows the configuration of the BSS via the mmWave interface (MMWave_config).
 *      It parses input CLI configuration commands like chirp profile and GUI configuration.
 *      When sensor start CLI command is parsed, it sends @ref MMWDEMO_CLI_SENSORSTART_EVT to
 *      @ref MmwDemo_sensorMgmtTask task and then waits for @ref MMWDEMO_START_COMPLETED_EVT before
 *      accepting new commands over the CLI/UART.
 *      When sensor stop CLI command is parsed, it sends @ref MMWDEMO_CLI_SENSORSTOP_EVT to
 *      @ref MmwDemo_sensorMgmtTask task and then waits for @ref MMWDEMO_STOP_COMPLETED_EVT before
 *      accepting new commands over the CLI/UART.
 *    - @ref MmwDemo_sensorMgmtTask. This task accepts sensor start and stop commands either via
 *      CLI or GPIO buttons on the EVMs. It performs the following steps on receiving the start and stop
 *      commands:
 *          - MMWDEMO_CLI_SENSORSTART_EVT:
 *              -# Issues MMWave_config using the
 *              previously parsed configurations to setup the BSS.
 *              -# Configures the only-once (@ref MmwDemo_dataPathConfigCommon) and
 *              first-time (@ref MmwDemo_config1D_HWA, @ref MmwDemo_dataPathTrigger1D)
 *              data path processing configurations, so that the processing chain is ready
 *              to do the first step of 1D processing related to chirps (see @ref datapath).
 *              -# Issues MMWave_start to command the BSS to start chirping.
 *              -# Switches on the @ref SOC_XWR14XX_GPIO_2 LED on EVM
 *              -# Signals the @ref MMWDEMO_START_COMPLETED_EVT event to CLI task. <br>
 *          - MMWDEMO_CLI_SENSORSTOP_EVT:
 *              -# Issues MMWave_stop to stop the BSS and the chirping
 *              -# Switches off the @ref SOC_XWR14XX_GPIO_2 LED on EVM
 *              -# Signals the @ref MMWDEMO_STOP_COMPLETED_EVT event to CLI task.
 *
 *     Internally, @ref MmwDemo_sensorMgmtTask has a state machine that handles CLI and key requests. The following
 *     diagram illustrates the state machine and how state changes upon different events. Also, BSS may trigger
 *     the sensor to stop, such as "finite loop frame".
 *
 *  @image html sensor_management.png
 *
 *    - @ref MmwDemo_mmWaveCtrlTask. This task is used to provide an execution
 *      context for the mmWave control, it calls in an endless loop the MMWave_execute API.
 *    - @ref MmwDemo_dataPathTask. The task performs in real-time:
 *      - Data path processing chain control and (re-)configuration
 *        of the hardware entities involved in the processing chain, namely HWA and EDMA.
 *      - Transmits the detected objects etc through the UART output port.
 *        For format of the data on UART output port, see @ref MmwDemo_transmitProcessedOutput.
 *        The UART transmission is done in the data path processing task
 *        itself although it could be done in a separate thread to potentially parallelize
 *        data path processing with transmission on UART. Presently, the UART processing
 *        is dominant because of slow UART speed and data path processing time on R4F CPU
 *        is not much (about 2 ms, most work done by HWA) hence this is not a problem.
 *        Separation of the transmit task may be done in future versions of the demo.
 *
 *  @section datapath Data Path - Overall
 *   @image html datapath_overall.png "Top Level Data Path Processing Chain"
 *   \n
 *   \n
 *   @image html datapath_overall_timing.png "Top Level Data Path Timing"
 *
 *   As seen in the above picture, the data path processing consists of:
 *   - Processing during the chirps as seen in the timing diagram:
 *     - This consists of 1D (range) FFT processing that takes input from multiple
 *     receive antennae from the ADC buffer for every chirp (corresponding to the
 *     chirping pattern on the transmit antennae)
 *     and performs FFT on it and generates transposed output into the L3 RAM.
 *     This is done using HWA and EDMA, more details of which can be seen in @ref data1d
 *   - Processing during the idle or cool down period of the RF circuitry following the chirps
 *     until the next chirping period, shown as "Inter frame processing time" in the timing diagram.
 *     This processing consists of:
 *     - 2D (velocity) FFT processing that takes input from 1D output in L3 RAM and performs
 *       FFT to give a (range,velocity) matrix in the L3 RAM. This is done using HWA and EDMA,
 *       more details of which can bbe seen in @ref data2d
 *     - CFAR detection using HWA. More details can be seen at @ref dataCFAR.
 *     - Post processing using R4F. More details can be seen at @ref dataPostProc
 *     - Direction of Arrival Estimation (Azimuth, Elevation). More details can be seen at
 *       @ref dataAngElev_low, @ref dataAngElev_high and @ref dataXYZ
 *
 *  @subsection antConfig Antenna Configurations
 *  The following figure shows antenna layout as seen from the front of
 *  the EVM xWR14xx board alongside the x,y,z coordinate convention.
 *   @image html antenna_design.png "xWR14xx Antenna layout"
 *
 *   As seen in figures below, the millimeter wave demo supports two antenna configurations:
 *     - Two transmit antennas and four receive antennas. Transmit antennas Tx1 and Tx3
 *       are horizontally spaced at d = 2 Lambda, with their transmissions interleaved in
 *       a frame. This configuration allows for azimuth estimation.
 *     - Three  transmit and four receive antennas . The third Tx antenna, Tx2, is
 *       positioned between the other two Tx antennas at half lambda elevation. This
 *       configuration allows for both azimuth and elevation estimation.
 *   \n
 *   @image html antenna_layout_2D.png "Antenna Configuration 2D (azimuth estimation)"
 *   \n
 *   @image html antenna_layout_3D.png "Antenna Configuration 3D (azimuth and elevation estimation)"
 *   \n
 *
 *   @subsection data1d Data Path - 1st Dimension FFT Processing
 *   @image html datapath_1d_elevation.png "Data Path 1D"
 *   \n
 *   @image html datapath_1d_timing.png "Data Path 1D timing diagram"
 *
 *   Above pictures illustrate a case of 3 *16 = 48 chips per frame and 225 samples per receive antenna per
 *   chirp with three transmit antennas as mentioned in @ref antConfig, chirping within the frame with
 *   repeating pattern of (Tx1,Tx3,Tx2).
 *   This is the 3D profile (velocity and x,y,z) case. There are 4 rx
 *   antennas, the samples of which are color-coded and labeled as 1,2,3,4 with
 *   unique coloring for each of chirps that are processed in ping-pong manner
 *   to parallelize accelerator and EDMA processing with sample acquisition from ADC.
 *   The hardware accelerator's parameter RAMs are setup to do 256 point FFTs
 *   which operates on the input ADC ping and pong buffers to produce output in M2 and M3
 *   memories of the HWA.
 *   Initially the software triggers (@ref MmwDemo_dataPathTrigger1D) the processing by activating HWA's dummy
 *   params PARAM_0 (ping) and PARAM_2 (pong) which in turn activate the processing PARAMs
 *   PARAM_1 (ping) and PARAM_3 (pong) which are waiting on the ADC full signal.
 *   When ADC has samples to process in the ADC buffer Ping or Pong memories, the corresponding
 *   processing PARAM will trigger FFT calculation and transfer the FFT output into the M2 or M3 memories.
 *   Before ADC samples are sent to FFT engine, a Blackman window is applied to them in the HWA.
 *   The completion of FFT also triggers EDMA which has been setup to do a copy with
 *   transposition from the M2/M3 memories to the L3 RAM (Radar Cube Matrix, @ref
 *   MmwDemo_DataPathObj::radarCube)
 *   as shown in the picture.
 *   This HWA-EDMA ping-pong processing is done 48/2(ping/pong) = 24 times so
 *   that all chirps of the frame are processed. The EDMA
 *   is setup so that after processing every chirp, the EDMA B or EDMA D which are
 *   chained from EDMA A and EDMA C channels will trigger the HWA's dummy PARAMs.
 *   The EDMA C in the picture is setup to give a completion interrupt after the last chirp
 *   which notifies software that 1D processing is complete and software can trigger
 *   1D processing again for the next chirping period when the time comes.
 *   The shadow (link) PaRAMs of EDMA are used for reloading the PaRAMs so reprogramming
 *   is avoided. The blue arrows between EDMA blocks indicate linking and red arrows
 *   indicate chaining.
 *
 *   In the above pictures:
 *   - A is @ref MMW_EDMA_1D_PING_CH_ID
 *   - B is @ref MMW_EDMA_1D_PING_CHAIN_CH_ID
 *   - A_shadow is @ref MMW_EDMA_1D_PING_SHADOW_LINK_CH_ID
 *   - B_shadow is @ref MMW_EDMA_1D_PING_ONE_HOT_SHADOW_LINK_CH_ID
 *   - C is @ref MMW_EDMA_1D_PONG_CH_ID,
 *   - D is @ref MMW_EDMA_1D_PONG_CHAIN_CH_ID
 *   - C_shadow is @ref MMW_EDMA_1D_PONG_SHADOW_LINK_CH_ID
 *   - D_shadow is @ref MMW_EDMA_1D_PONG_ONE_HOT_SHADOW_LINK_CH_ID
 *
 *  @subsection data2d Data Path - Lower Precision 2nd Dimension Processing
 *   @image html datapath_2d_top_level.png "Data Path 2D FFT high level diagram"
 *   \n
 *   \n
 *   @image html datapath_2d_timing.png "Data Path 2D FFT timing diagram"
 *
 *
 *   As shown in the high level diagram above, the 2D processing performs
 *   a 2nd dimension (Doppler) FFT on the range data from 1D output, the processing is
 *   done in a ping-pong manner. It consists of the following steps:
 *   -# Static clutter removal if enabled. For each range bin, per each antenna, the mean value of
 *   the samples is calculated and subtracted from the samples. The operation is
 *   performed by R4F directly on data in L3.
 *   -# The data from the L3 RAM (@ref MmwDemo_DataPathObj::radarCube) is transferred by EDMA into
 *   the M0 (even pair or ping) and M1 (odd pair or pong) memories of the HWA which then
 *   performs the 2D-FFT and produces the output in M2 and M3 memories. Before FFT operation,
 *   input samples are multiplied by a window function.
 *   -# The HWA performs log magnitude operation on M2 and M3 memories from above step
 *   and produces results in M0 and M1.
 *   -# The HWA performs the sum operation on the M0 and M1 memories from above step
 *   and produces results in the M2 and M3 memories appended to results from step 1.
 *   -# The EDMA copies the results from step 1 to the L3 RAM at
 *    @ref MmwDemo_DataPathObj::radarCube (in the same place as the input in step 1)
 *    and then the EDMA copies the
 *    results from the previous step to the
 *    L3 RAM at @ref MmwDemo_DataPathObj::rangeDopplerLogMagMatrix.
 *
 *   The data is transferred and processed in chunks of @ref MMW_NUM_RANGE_BINS_PER_TRANSFER rows.
 *   The timing of ping-pong parallelism is shown in the timing diagram above. For more details
 *   of the data flow like the format of data in the memories between stages,
 *   the EDMA and HWA resources, etc, refer to the detailed diagram below.
 *   @image html datapath_2d_detailed_elevation.png "Data Path 2D FFT  detailed diagram"
 *
 *   In the detailed diagram:
 *   - A is @ref MMW_EDMA_2D_PING_CHAIN_CH_ID2,
 *   - B is @ref MMW_EDMA_2D_PING_CHAIN_CH_ID3
 *   - A_shadow is @ref MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID3
 *   - B_shadow is @ref MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID4
 *   - C is @ref MMW_EDMA_2D_PONG_CHAIN_CH_ID2,
 *   - D is @ref MMW_EDMA_2D_PONG_CHAIN_CH_ID3
 *   - C_shadow is @ref MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID3
 *   - D_shadow is @ref MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID4
 *   - E is @ref MMW_EDMA_2D_PING_CH_ID,
 *   - F is @ref MMW_EDMA_2D_PING_CHAIN_CH_ID1 (chained to A)
 *   - E_shadow is @ref MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID1
 *   - F_shadow is @ref MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID2
 *   - G is @ref MMW_EDMA_2D_PONG_CH_ID,
 *   - H is @ref MMW_EDMA_2D_PONG_CHAIN_CH_ID1 (chained to C)
 *   - G_shadow is @ref MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID1
 *   - H_shadow is @ref MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID2
 *
 *   The 2D processing is triggered by software by starting EDMA A and EDMA C.
 *   (@ref MmwDemo_dataPathTrigger2D).
 *   \n
 *   \n
 *
 *  @subsection data2dCombined2DandLogMag Data Path - Higher precision 2nd Dimension Processing
 *   In the data path processing described above (@ref data2d), the FFT output is of
 *   24-bit precision but is converted to 16-bit for storage in radar Cube and this 16-bit is also used for
 *   log2 processing. Reason for this scheme is limited storage space in radar cube. However, this loss of
 *   precision results in spiky noise profile which increases false detections in CFAR. One way to solve
 *   this problem would be to store output of FFT in the M memory as 32-bit instead of 16-bit before log2 operation.
 *   However, this requires storage (in bytes) = 3 (num tx antennas) * 4 (num rx antennas) * 8 (complex 32-bit) * Doppler FFT size.
 *   This must fit in the 16 KB size for each of the four M memories, which limits the Doppler FFT size to 512, but
 *   we support up to 1024. Hence we have to choose the option in HWA to chain the FFT and log magnitude directly without
 *   going to M memory, this retains the 24-bit precision. However this comes at the cost of recomputing the 2D
 *   FFT during DOA calculation. So it is a trade-off between precision and MIPS. By default, this
 *   higher precision scheme is selected but user can change to the lower precision scheme
 *   by changing the following line of code in main.c to set the left hand side variable
 *   to @ref DATA_PATH_CHAIN_SEPARATE_LOGMAG.
 *   @verbatim
       gMmwMCB.dataPathObj.datapathChainSel = DATA_PATH_CHAIN_COMBINED_LOGMAG;
     @endverbatim
 *
 *   The following diagram shows the modified data Path 2D FFT.
 *   Since 2D FFT output is not stored in Radar Cube, the Radar Cube still contains 1D FFT data.
 *
 *   @image html datapath_2d_detailed_combinedFFTandLogMagnitude.png "Data Path 2D FFT  detailed diagram - combined 2D FFT and Log-Magnitude"
 *
 *
 *   In the detailed diagram:
 *   - A is @ref MMW_EDMA_2D_PING_CHAIN_CH_ID2,
 *   - B is @ref MMW_EDMA_2D_PING_CHAIN_CH_ID3
 *   - A_shadow is @ref MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID3
 *   - B_shadow is @ref MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID4
 *   - C is @ref MMW_EDMA_2D_PONG_CHAIN_CH_ID2,
 *   - D is @ref MMW_EDMA_2D_PONG_CHAIN_CH_ID3
 *   - C_shadow is @ref MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID3
 *   - D_shadow is @ref MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID4
 *   - E is @ref MMW_EDMA_2D_PING_CH_ID (chained to A),
 *   - E_shadow is @ref MMW_EDMA_2D_PING_SHADOW_LINK_CH_ID1
 *   - G is @ref MMW_EDMA_2D_PONG_CH_ID (chained to C),
 *   - G_shadow is @ref MMW_EDMA_2D_PONG_SHADOW_LINK_CH_ID1
 *
 *   The 2D processing is triggered by software by starting EDMA A and EDMA C.
 *   (@ref MmwDemo_dataPathTrigger2D).
 *   \n
 *   \n
 *
 *  @subsection dataCFAR Data Path - CFAR Detection
 *   @image html datapath_cfar.png "Data Path CFAR Detection Diagram"
 *   \n
 *  As shown in the above picture, CFAR processing consists of:
 *  -# The software triggers (@ref MmwDemo_dataPathTriggerCFAR)
 *     the EDMA that transfers the range-doppler log magnitude matrix from L3 RAM
 *     @ref MmwDemo_DataPathObj::rangeDopplerLogMagMatrix (output of 2D processing)
 *     to the M0 memory.
 *  -# The HWA performs CFAR computation in M0 and produces output in M2 memory and
 *     a CFAR completion interrupt from HWA is generated to the R4F CPU.
 *
 *  In the above picture:
 *  - A is @ref MMW_EDMA_CFAR_INP_CH_ID
 *  - B is @ref MMW_EDMA_CFAR_INP_CHAIN_CH_ID
 *  - A_Shadow is @ref MMW_EDMA_CFAR_INP_SHADOW_LINK_CH_ID1
 *  - B_Shadow is @ref MMW_EDMA_CFAR_INP_SHADOW_LINK_CH_ID2
 *
 * The following are default CFAR configuration parameters:
 *  - @ref MMW_HWA_NOISE_AVG_MODE
 *  - @ref MMW_HWA_CFAR_THRESHOLD_SCALE
 *  - @ref MMW_HWA_CFAR_WINDOW_LEN
 *  - @ref MMW_HWA_CFAR_GUARD_LEN
 *  - @ref MMW_HWA_CFAR_NOISE_DIVISION_RIGHT_SHIFT
 *  - @ref MMW_HWA_CFAR_PEAK_GROUPING
 *
 * These parameters can be changed using a cli configuration command cfarCfg.
 * The command with the arguments are described below:
 *
 * cfarCfg \<averageMode\> \<winLen\> \<guardLen\> \<noiseDiv\> \<cyclicMode\> \<peakGrouping\> \<thresholdScale\>
 *
 * where
 *  - \<averageMode\>     - 0-CFAR_CA, 1-CFAR_CAGO, 2-CFAR_CASO
 *  - \<winLen\>          - CFAR Noise averaging window length
 *  - \<guardLen\>        - CFAR guard length
 *  - \<noiseDiv\>        - CFAR noise averaging divisor (right shift value)
 *  - \<cyclicMode\>      - 0-cyclic mode disabled, 1-cyclic mode enabled
 *  - \<peakGrouping\>    - 0-peak grouping disabled, 1-peak grouping enabled
 *  - \<thresholdScale\>  - Detection scale factor
 *
 *  @subsection dataPostProc Data Path - Post processing
 *    Post processing includes peak grouping and Doppler phase shift compensation.
 *
 *    Peak grouping function  (@ref MmwDemo_peakGrouping) is done by the R4F CPU after CFAR processing.
 *    Its inputs are:
 *    - List of detected objects by CFAR Detection from the 2D output in M2.
 *      Each detected object is described by three parameters: range index, Doppler index, and noise
 *      energy in CFAR cell, as seen in @ref cfarDetOutput_t.
 *    - Radar cubed matrix, located in L3 memory (@ref MmwDemo_DataPathObj::radarCube).
 *    - Log magnitude range Doppler matrix in L3 memory (@ref MmwDemo_DataPathObj::rangeDopplerLogMagMatrix).
 *
 *    The function performs the following:
 *    -# Discards detected objects with range indices outside of the range specified by peakGrouping CLI command.
 *    -# Discards detected objects whose FFT peaks in the range-doppler matrix
 *     (@ref MmwDemo_DataPathObj::rangeDopplerLogMagMatrix) are smaller than its neighbors.
 *    -# For each selected object, copies the 2nd Dimensional FFT complex values of received virtual antennas
 *      located in @ref MmwDemo_DataPathObj::radarCube to M0 (azimuth antennas) and M1 (elevation antennas)
 *      for further azimuth and elevation FFT calculation.
 *    -# For each selected object, copies its (range, Doppler) indices to R4F CPU's local memory
 *       @ref MmwDemo_DataPathObj::objOut, which will be eventually shipped out after the azimuth
 *       and x,y,z calculations are done (refer to @ref dataXYZ).
 *
 *    Doppler compensation function  (@ref MmwDemo_dopplerCompensation) is done by the R4F CPU after CFAR and peak grouping processing.
 *
 *    Its inputs are separate Azimuth and Elevation symbol arrays.
 *
 *    It performs compensation for the Doppler phase shift on the symbols corresponding to the virtual Rx antennas. In case of 2Tx MIMO scheme, the second set of Rx symbols
 *       is rotated by half of the estimated Doppler phase shift between subsequent chirps corresponding to the same Tx antenna. In case of 3Tx MIMO elevation scheme, the second set
 *       of Rx symbols is rotated by third of the estimated Doppler phase shift, while the third set of Rx symbols corresponding to the third Tx antenna is rotated by 2/3 of the estimated Doppler phase shift.
 *       Refer to the pictures below.
 *    @anchor Figure_doppler
 *    @image html angle_doppler_compensation.png "Figure_doppler: Doppler Compensation"
 *
 *   At the end, the function returns the number of selected objects.
 *
 *  @subsection dataAngElev_low Data Path - Direction of Arrival FFT Calculation corresponding to Lower Precision 2D Processing
 *    @image html datapath_azimuth_fft.png
 *
 *    Azimuth/elevation FFT computation is triggered by the software (@ref MmwDemo_dataPathTriggerAngleEstimation)
 *    if the number of detected  peaks after the post processing stage is greater than zero. For each
 *    detected object in M0 (azimuth antennas) and M1 (elevation antennas), it performs the following steps:
 *    -# Complex FFT of the array of elevation antennas from M1 to M3.
 *    -# Complex FFT of the array of azimuth antennas from M0 to M2.
 *    -# Log magnitude FFT of the array azimuth antennas from M0 to M1 (over-writes the input of step 1). This step
 *       is performed for the purpose of finding the peak. The position of the peak is used to look-up the output of
 *       calculations in above two steps to obtain the azimuth and elevation peak points which are referred in
 *       @ref dataXYZ as \f$P_1\f$ and \f$P_2\f$ respectively.
 *
 *    Currently the size of FFT is hardcoded and defined by @ref MMW_NUM_ANGLE_BINS.
 *    If the number of Tx elevation antennas is equal to zero (no elevation), only step 2 above is done.
 *
 *  @subsection dataAngElev_high Data Path - Direction of Arrival FFT Calculation corresponding to Higher Precision 2D Processing
 *
 *    @image html datapath_alternative.png
 *
 *    For higher precision 2D processing, recall that in 2D FFT phase, \n
 *        - HWA calculates log Magnitude together with 2D FFT without output formating
 *          into 16 bits data to preserve resolution for CFAR detection.
 *        - It saves only rangeDoppler Matrix and leaves radar cube filled with 1D FFT data.
 *
 *    After CFAR and peak grouping phase, \n
 *        - 2D FFT is recalculated for the detected objects.
 *        - The rest of the Angle estimation and Elevation estimation are the same as in @ref dataAngElev_low.
 *
 *
 *   In the diagram:
 *   - A is @ref MMW_EDMA_2DFFT_SINGLERBIN_CH_ID,
 *   - B is @ref MMW_EDMA_2DFFT_SINGLERBIN_CHAIN_CH_ID
 *   - A_shadow is @ref MMW_EDMA_2DFFT_SINGLERBIN_SHADOW_LINK_CH_ID
 *   - B_shadow is @ref MMW_EDMA_2DFFT_SINGLERBIN_SHADOW_LINK_CH_ID2
 *
 *  @subsection dataXYZ Data Path - Direction of Arrival Estimation (x,y,z)
 *    This processing is done on R4F CPU in the function @ref angleEstimationAzimElev.
 *    \anchor Figure_geometry
 *    @image html coordinate_geometry.png "Figure A: Coordinate Geometry"
 *    \n
 *    \anchor Figure_wz
 *    @image html coordinate_geometry_wz.png "Figure wz"
 *    \n
 *    \anchor Figures_wx
 *    @image html coordinate_geometry_wx.png "Figures wx"
 *    \ref Figure_geometry shows orientation of x,y,z axes with respect to the sensor/antenna positions. The objective is to estimate
 *    the (x,y,z) coordinates of each detected object.
 *    \f$w_x\f$ is the phase difference between consecutive receive azimuth antennas of the 2D FFT and \f$w_z\f$ is the phase difference between
 *    azimuth and corresponding elevation antenna above the azimuth antenna. The phases for each antenna are shown in the \ref Figure_doppler.
 *    \ref Figure_wz shows that the distance AB which represents the relative distance between wavefronts intersecting consecutive elevation
 *    antennas is \f$AB = \frac{\lambda}{2} \sin (\phi)\f$. Therefore \f$W_z = \frac{2\pi}{\lambda} \cdot AB\f$, therefore \f$W_z = \pi \sin (\phi)\f$.
 *    Note that the phase of the lower antenna is advanced compared to the upper
 *    antenna which is why picture X shows -Wz term in the upper antenna.
 *    \ref Figures_wx show that distance CD which represents the relative distance between wavefronts intersecting consecutive azimuth
 *    antennas is \f$CD = \frac{\lambda}{2} \sin (\theta) \cos (\phi)\f$ Therefore \f$w_x =  \frac{2\pi}{\lambda} \cdot CD\f$,
 *    therefore \f$w_x = \pi \sin (\theta) \cos (\phi)\f$.
 *    For a single obstacle, the signal at the 8 azimuth antennas will be (\f$A_1\f$ and \f$\psi\f$ are the arbitrary starting
 *    amplitude/phase at the first antenna):
 *    \f[
 *    A_1 e^{j\psi} [ 1 \; e^{jw_x} \; e^{j2w_x} \; e^{j3w_x} \; e^{j4w_x} \; e^{j5w_x} \; e^{j6w_x} \; e^{j7w_x} ]
 *    \f]
 *
 *    An FFT of the above signal will yield a peak \f$P_1\f$ at \f$w_x\f$, with the phase of this peak being \f$\psi\f$ i.e
 *    \f[
 *       P_1 = A_1 e^{j\psi}
 *    \f]
 *    If \f$k_{MAX}\f$ is the index of the peak in log magnitude FFT represented as
 *    signed index in range \f$[-\frac{N}{2}, \frac{N}{2}-1]\f$, then \f$ w_x \f$ will be
 *       \f[
 *            w_x = \frac{2\pi}{N}k_{MAX}
 *       \f]
 *
 *    The signal at the 4 elevation antennas will be:
 *     \f[
 *      A_2 e^{j(\psi + 2 w_x - w_z)} [ 1 \; e^{jw_x} \; e^{j2w_x} \; e^{j3w_x}]
 *     \f]
 *
 *    An FFT of the above signal will yield a peak \f$P_2\f$ at \f$w_x\f$, with the phase of this peak being \f$\psi + 2w_x - w_z\f$.
 *     \f[
 *        P_2 = A_2 e^{j(\psi+ 2 w_x - w_z)}
 *     \f]
 *
 *    From above,
 *     \f[
 *        P_1 \cdot P_2^* = A_1 \cdot A_2 e^{j(\psi - (\psi+ 2 w_x - w_z))}
 *     \f]
 *
 *    Therefore,
 *     \f[
 *       w_z=\angle (P_1 \cdot P_2^* \cdot e^{j2w_x})
 *     \f]
 *
 *
 *    Calculate range (in meters) as:
 *       \f[
 *           R=k_r\frac{c \cdot F_{SAMP}}{2 \cdot S \cdot N_{FFT}}
 *       \f]
 *       where, \f$c\f$ is the speed of light (m/sec), \f$k_r\f$ is range index, \f$F_{SAMP}\f$ is the sampling frequency (Hz),
 *       \f$S\f$ is chirp slope (Hz/sec), \f$N_{FFT}\f$ is 1D FFT size.
 *    Based on above calculations of \f$R\f$, \f$w_x\f$ and \f$w_z\f$, the \f$(x,y,z)\f$ position of the object
 *    can be calculated as seen in the \ref Figure_geometry,
 *       \f[
 *           x = R\cos(\phi)\sin(\theta) = R\frac{w_x}{\pi}, z = R\sin(\phi) = R\frac{w_z}{\pi}, y = \sqrt{R^2-x^2-z^2}
 *       \f]
 *    The computed \f$(x,y,z)\f$ and azimuth peak for each object are populated in their
 *    respective positions in @ref MmwDemo_DataPathObj::objOut which is
 *    then shipped out on the UART port (@ref MmwDemo_transmitProcessedOutput).
 *    To be able to detect two objects at the same range-doppler index but at different angle,
 *    search for the 2nd peak in the azimuth FFT and compare its height relative to the first peak height,
 *    and if detected, create new object in the list with the same range/Doppler
 *    indices, and repeat above steps to calculate (x,y,z) coordinates. To enable/disable
 *    the two peak detection or to change the threshold for detection, refer to
 *    @ref MMWDEMO_AZIMUTH_TWO_PEAK_DETECTION_ENABLE and @ref MMWDEMO_AZIMUTH_TWO_PEAK_THRESHOLD_SCALE.
 *
 *  @subsection output Output information sent to host
 *      Please refer to this section in the doxygen documentation of the xwr16xx mmw Demo because
 *      this is part of a unified interface. For the stats information, the
 *      following diagram applies for 14xx.
 *
 *      @image html margins_xwr14xx.png "Margins and R4F CPU loading"
 *
 *  @subsection calibration Range Bias and Rx Channel Gain/Offset Measurement and Compensation
 *       Please refer to this section in the doxygen documentation of the xwr16xx mmw Demo.
 *       The differences are as follows:
 *       -# For the scheme @ref DATA_PATH_CHAIN_COMBINED_LOGMAG, because we are
 *         computing azimuth static heat map on 1D FFT (a limitation that will be removed
 *         in a future release), the calibration procedure will
 *         not be immune to motion in the scene, hence during calibration caution
 *         must exercised to ensure there are no moving objects in the background.
 *       -# The azimuth static heat map itself is not corrected, this limitation
 *         will be removed in a future release.
 *
 *  @subsection datapathConfig Data Path - Notes on configuration and synchronization.
 *    - EDMA: For all of the data path processing, the demo uses non-overlapping
 *      EDMA resources (chain channels, link parameter sets etc) so that all EDMA configuration
 *      only needs to be done once (@ref MmwDemo_config1D_EDMA, @ref MmwDemo_config2D_EDMA and
 *      @ref MmwDemo_configCFAR_EDMA -- called in @ref MmwDemo_dataPathConfigCommon). The channel
 *      resources involved in chaining are using the channels that are not tied to any hardware
 *      entity on the SoC, these are ones that have suffix EDMA*_FREE_\<n\> as specified in the
 *      @ref sys_common.h file. An alternative implementation that is more EDMA
 *      resource constrained may overlap resources among 1D, 2D and CFAR processing at the
 *      cost of processing cycles to reconfigure the EDMA in real-time. The R4F CPU
 *      programs the EDMA through the utility APIs in @ref config_edma_util.h, which
 *      in turn invoke the EDMA driver configuration APIs to program the EDMA.
 *      The R4F CPU gets notified of the channels on which it desires completion notification
 *      through an application call-back function provided to the EDMA driver and
 *      in this function it posts the semaphore on which it can wait for completion.
 *      Present implementation shows a single call back function (@ref MmwDemo_EDMA_transferCompletionCallbackFxn)
 *      which posts different semaphores (@ref MmwDemo_DataPathObj::EDMA_1Ddone_semHandle,
 *      @ref MmwDemo_DataPathObj::EDMA_2Ddone_semHandle and @ref MmwDemo_DataPathObj::EDMA_CFARdone_semHandle)
 *      based on which channel (or rather transfer completion codes) were
 *      indicated complete by the EDMA driver. The R4F CPU can pend on the semaphores
 *      in the data path processing chain.
 *    - HWA: The window RAMs are configured for the 1D and 2D FFT only once
 *      (in @ref MmwDemo_dataPathConfigCommon). The 1D window is currently chosen to be
 *      Blackman and the 2D is chosen to be Hanning window. This may be made configurable
 *      in future. Currently, HWA PARAMs are non-overlapping among the various processing stages.
 *      So they could be configured only once like the EDMA case, but current code shows them
 *      being re-configured in the real-time along with setting the start,end indices and
 *      the loop count required for each processing stage
 *      (@ref MmwDemo_config1D_HWA, @ref MmwDemo_config2D_HWA, @ref MmwDemo_configCFAR_HWA
 *      and @ref MmwDemo_configAngleEstimation_HWA).
 *      The R4F CPU gets notified of HWA completion through an application supplied
 *      semaphore (@ref MmwDemo_DataPathObj::HWA_done_semHandle) to the HWA driver
 *      and it can pend on this semaphore. There are utility
 *      APIs provided in @ref config_hwa_util.h that are used to
 *      by the application to program the HWA.
 *
 *    @subsection designNotes Data Path Design Notes
 *    @subsubsection L3 Heap
 *      The data buffer arrangement in L3 heap is shown in the following diagram.
 *      The buffers are allocated through @ref MmwDemo_memPoolAlloc. The size of each segment
 *      is determined at runtime based on configurations.
 *      - radarCube size = numRangeBins * numDopplerBins * numTxAntennas * numRxAntennas * 4 bytes
 *      - rangeDopplerLogMagMatrix Size = numRangeBins * numDopplerBins * 2 bytes
 *      - cfarDetectionOut Size = @ref MMW_MAX_OBJ_OUT * sizeof(@ref cfarDetOutput_t)
 *      - azimuthStaticHeatMap Size = numRangeBins *  numVirtualAntAzim * 4 bytes (if azimuth heatmap is enabled)
 *
 *      @image html data_buffers_layout.png "Data buffers layout arrangement".
 *
 *    @subsubsection scaling Scaling
 *      The HWA uses 24-bit fixed point arithmetic for the data path processing.
 *      In order to prevent overflows in the FFT processing, the scaling factors have to be
 *      set appropriately in the HWA configuration. The HWA has up to 10 stages of processing
 *      with ability to scale by 1/2 for each stage.
 *      - 1D processing: If the HWA's FFT scale is set to \f$\frac{1}{2^k}\f$
 *      where \f$k\f$ is the number of stages for which the scaling is enabled, and
 *      input to the FFT were a pure tone at one of the bins, then
 *      the output magnitude of the FFT at that bin will be \f$\frac{N}{2^k}\f$ (\f$N\f$ is the FFT order) times
 *      the input tone amplitude (because tone is complex, this implies that the individual real and
 *      imaginary components will also be amplified by a maximum of this scale).
 *      Because we do a Blackman window before the FFT, the overall scale is about 1/2.4 of the FFT scale.
 *      This means for example for 256 point FFT, the windowing + FFT scale will be \f$\frac{106.7}{2^k}\f$.
 *      For k=2 which is currently how it is in the implementation (no matter the FFT order), this will be 26.7.
 *      Therefore, the ADC output when it is a pure tone should not exceed +/-2^15/26.7 = 1228 for
 *      the I and Q components (even though HWA is internally 24-bit, the FFT output is stored
 *      as 16-bit before 2D processing, hence 2^15).
 *      The XWR14xx EVM when presented with a strong single reflector
 *      reasonably close to it (with Rx dB gain of 30 dB in the chirp profile)
 *      shows ADC samples to be a max of about 2000 and while this exceeds
 *      the 1228 maximum, is not a pure tone, the energy of the FFT is seen in other bins
 *      also and the solution still works well and detects the strong object.
 *      - 2D processing: For the 2D FFT, given that the input is the output of 1D FFT that
 *      can amplify its input as mentioned in previous section, it is more appropriate to use
 *      full scale. So currently in the implementation, scaling is enabled for all stages of the FFT
 *      except for the first stage because of Hanning window related scaling by 1/2.
 *
 *    @subsubsection lvds LVDS Streaming
 *      LVDS Streaming is unverified code. LVDS streaming conflicts with data path processing and should not be used.
 *
 *  @section memoryUsage Memory Usage
 *  @subsection memUsageSummary Memory usage summary
 *    The table below shows the usage of various memories available on the device across
 *    the demo application and other SDK components. The table is generated using the demo's
 *    map file and applying some mapping rules to it to generate a condensed summary.
 *    For the mapping rules, please refer to <a href="../../demo_mss_mapping.txt">demo_mss_mapping.txt</a>.
 *    The numeric values shown here represent bytes.
 *    Refer to the <a href="../../xwr14xx_mmw_demo_mss_mem_analysis_detailed.txt">xwr14xx_mmw_demo_mss_mem_analysis_detailed.txt</a>
 *    for detailed analysis of the memory usage across drivers and control/alg components and to
 *    <a href="../../demo_mss_mapping_detailed.txt">demo_mss_mapping_detailed.txt</a> for detailed mapping rules .
 *
 *  \include xwr14xx_mmw_demo_mss_mem_analysis.txt
 *
 *    Note on L3 memory and overlay
 *    ------------------------------
 *    A quick look at the L3_SRAM column will show that the total of that column exceeds the total
 *    physical memory available on the device. The reason is that we use the code-data overlay mechanism
 *    to virtually extend the available memory on the device. One-time startup code is overlaid with the
 *    radar cube. At startup, the application code accesses these functions to perform one-time setup functionality.
 *    Beyond that point, application code does not have a need to access these functions again and hence switches
 *    to access radarCube placed at the exact same location. Refer to the linker command file of the demo on the
 *    mechanics of this overlay technique.
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
#include <ti/sysbios/utils/Load.h>


/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/demo/io_interface/mmw_output.h>

#include "config_edma_util.h"
#include "config_hwa_util.h"
#include "post_processing.h"

/* Demo Include Files */
#include "mmw.h"
#include "data_path.h"
#include "mmw_lvds_stream.h"
#include <ti/demo/io_interface/mmw_config.h>
#include <ti/demo/utils/rx_ch_bias_measure.h>
#include <ti/demo/utils/mmwDemo_monitor.h>

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET        0U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET         0x400U

extern mmwHwaBuf_t gMmwHwaMemBuf[MMW_HWA_NUM_MEM_BUFS];

extern uint32_t log2Approx(uint32_t x);

/*! L3 RAM buffer */
uint8_t gMmwL3[SOC_XWR14XX_MSS_L3RAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram");

/* Data memory for CQ:Rx Saturation - 16 bit CQ  format */
rlRfRxSaturationCqData_t gCQRxSatMonMemory;

/* Data memory for CQ:Signal & Image band monitor  - 16 bit CQ format */
rlRfSigImgPowerCqData_t gCQRxSigImgMemory;

/*! L3 heap for convenience of partitioning L3 RAM */
MmwDemoMemPool_t gMmwL3heap =
{
    &gMmwL3[0],
    SOC_XWR14XX_MSS_L3RAM_SIZE,
    0
};

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB    gMmwMCB;


/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern void MmwDemo_CLIInit (void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);

void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathConfig(void);
void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathStop (MmwDemo_DataPathObj *obj);

void MmwDemo_getAngleBinsAtPeak(uint32_t numObj,
                                     MmwDemo_detectedObj *objOut,
                                     uint16_t *pAngleBins);

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                    MmwDemo_DataPathObj *obj);

void MmwDemo_initTask(UArg arg0, UArg arg1);
void MmwDemo_dataPathTask(UArg arg0, UArg arg1);
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Get a handle for ADCBuf.
 */
void MmwDemo_ADCBufOpen(MmwDemo_DataPathObj *obj)
{
    ADCBuf_Params       ADCBufparams;
    /*****************************************************************************
     * Start ADCBUF driver:
     *****************************************************************************/
    /* ADCBUF Params initialize */
    ADCBuf_Params_init(&ADCBufparams);
    ADCBufparams.chirpThreshold = 1;
    ADCBufparams.continousMode  = 0;

    /* Open ADCBUF driver */
    obj->adcbufHandle = ADCBuf_open(0, &ADCBufparams);
    if (obj->adcbufHandle == NULL)
    {
        //System_printf("Error: Unable to open the ADCBUF driver\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: ADCBUF Instance(0) %p has been opened successfully\n", obj->adcbufHandle);
}



/**
 *  @b Description
 *  @n
 *      Configures ADCBuf and returns the number of RxAntennas
 */
int32_t MmwDemo_ADCBufConfig(MmwDemo_DataPathObj *dataPathObj)
{

    ADCBuf_dataFormat   dataFormat;
    ADCBuf_RxChanConf   rxChanConf;
    uint8_t             channel;
    int32_t             retVal = 0;
    uint8_t             numBytePerSample = 0;
    MmwDemo_ADCBufCfg*  ptrAdcbufCfg;
    uint32_t            chirpThreshold;
    uint32_t            rxChanMask = 0xF;

    ptrAdcbufCfg = &dataPathObj->cliCfg->adcBufCfg;

    /* Check if ADC configuration is supported:*/
    /* ADC out bits: must be 16 Bits */
    MmwDemo_debugAssert(gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcBits == 2);
    
    /* ADC data format: must be complex */
    /*adcCfg command*/
    if((gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt != 1) &&
       (gMmwMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt != 2))
    {
        MmwDemo_debugAssert(0);
    }    
    /*adcbufCfg command*/
    MmwDemo_debugAssert(ptrAdcbufCfg->adcFmt == 0);
    
    /* ADC channel interleave mode: must be interleaved */
    MmwDemo_debugAssert(ptrAdcbufCfg->chInterleave == 0);

    
    /*****************************************************************************
     * Disable all ADCBuf channels
     *****************************************************************************/
    if ((retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
    {
       //System_printf("Error: Disable ADCBuf channels failed with [Error=%d]\n", retVal);
       MmwDemo_debugAssert (0);
       goto exit;
    }

    /* Complex dataFormat has 4 bytes */
    numBytePerSample =  4;

    /* Configure ADC buffer data format */
    dataFormat.adcOutFormat       = ptrAdcbufCfg->adcFmt;
    dataFormat.sampleInterleave   = ptrAdcbufCfg->iqSwapSel;
    dataFormat.channelInterleave  = ptrAdcbufCfg->chInterleave;

    /* Debug Message: */
    /*System_printf("Debug: Start ADCBuf driver dataFormat=%d, sampleSwap=%d, interleave=%d, chirpThreshold=%d\n",
                   dataFormat.adcOutFormat, dataFormat.sampleInterleave, dataFormat.channelInterleave,
                   ptrAdcbufCfg->chirpThreshold);*/

    retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)&dataFormat);
    if (retVal < 0)
    {
        MmwDemo_debugAssert (0);
        goto exit;
    }

    memset((void*)&rxChanConf, 0, sizeof(ADCBuf_RxChanConf));

    /* Enable Rx Channel indicated in channel configuration */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(gMmwMCB.cfg.openCfg.chCfg.rxChannelEn & (0x1<<channel))
        {
            /* Populate the receive channel configuration: */
            rxChanConf.channel = channel;
            retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
            if (retVal < 0)
            {
                MmwDemo_debugAssert (0);
                goto exit;
            }
            rxChanConf.offset  += dataPathObj->numAdcSamples * numBytePerSample;
        }
    }

    chirpThreshold = ptrAdcbufCfg->chirpThreshold;

    /* Set the chirp threshold: */
    retVal = ADCBuf_control(dataPathObj->adcbufHandle, ADCBufMMWave_CMD_SET_CHIRP_THRESHHOLD,
                            (void *)&chirpThreshold);
    if(retVal < 0)
    {
        MmwDemo_debugAssert (0);
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      EDMA transfer completion callback for CQ transfers
 *
 *  @param[in] arg                      Transfer completion callback argument
 *  @param[in] transferCompletionCode   Transfer completion code
 *
 *  @retval
 *      None
 */
void MmwDemo_EDMA_CQTransferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode)
{
    MmwDemo_DataPathObj *obj = (MmwDemo_DataPathObj *)arg;

    switch (transferCompletionCode)
    {
        case MMW_EDMA_RXSAT_TRANSFER_COMPLETION:
            /* Chained EDMA channels complete on 2nd EDMA channel */
            if(obj->datapathCQ.anaMonCfg->sigImgMonEn)
            {
                obj->datapathCQ.sigImgBandEdmaCnt++;
                MmwDemo_debugAssert(*(uint8_t *)obj->datapathCQ.sigImgData <= obj->datapathCQ.sigImgMonCfg->numSlices);
            }
            if(obj->datapathCQ.anaMonCfg->rxSatMonEn)
            {
                obj->datapathCQ.rxSatEdmaCnt++;
                MmwDemo_debugAssert(*(uint8_t *)obj->datapathCQ.rxSatData <= obj->datapathCQ.rxSatMonCfg->numSlices);
            }
            
        break;
        
        default:
            MmwDemo_debugAssert(0);
        break;
    }
}


/**
 *  @b Description
 *  @n
 *      Configures all CQ EDMA channels and param sets used in data path processing
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      -1 if error, 0 for no error
 */
int32_t MmwDemo_dataPathConfigCQEdma(MmwDemo_DataPathObj *obj)
{
    uint32_t    eventQueue;
    int32_t     retVal = 0;
    uint16_t    aCnt;

    /*****************************************************
     * EDMA configuration for getting CQ data from CQ buffer
     * to Datapath CQ storage
     *****************************************************/
    /* Use event queue 1 for CQ because when event queue 0 is used for CQ 
       and LVDS streaming is enabled (LVDS streaming uses event queue 0)
       then there is data corruption on the saved CQ data that is EDMAed below.
       Problem being investigated.*/       
    eventQueue = 1U;

    /*
        EDMA configurations for 2 chained EDMA channels:
        sigImgMonEn = 1, rxSatMonEn = 1,
            acnt: sigImgMonTotalSize, satMonTotalSize
        sigImgMonEn = 1, rxSatMonEn = 0,
            acnt: sigImgMonTotalSize, 0
        sigImgMonEn = 0, rxSatMonEn = 1,
            acnt: 0, satMonTotalSize     
    */    
    if(obj->datapathCQ.anaMonCfg->sigImgMonEn)
    {
        aCnt = obj->datapathCQ.sigImgMonTotalSize;
    }
    else
    {
        /* This becomes a dummy paRAM set */
        aCnt = 0;
    }

    /* Configure EDMA channel for sigImg monitor */
    retVal = EDMAutil_configSyncAwithChaining(obj->edmaHandle,
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.sigImgMonAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.sigImgData,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        EDMA_TPCC0_REQ_DFE_CHIRP_AVAIL, 
        MMW_EDMA_CH_RX_SATURATION_MON,
        true,
        EDMA_TPCC0_REQ_DFE_CHIRP_AVAIL,
        aCnt,
        1,
        0,
        0,
        eventQueue,
        false, /*isFinalTransferInterruptEnabled */
        true,  /* isFinalChainingEnabled */
        NULL,
        (uintptr_t) NULL);
        
    if (retVal != EDMA_NO_ERROR)
    {
        return -1;
    }

    if(obj->datapathCQ.anaMonCfg->rxSatMonEn)
    {
        aCnt = obj->datapathCQ.satMonTotalSize;
    }
    else
    {
        /* This becomes a dummy paRAM set */
        aCnt = 0;
    }
    
    /* Configure EDMA channel for RX saturation monitor */
    retVal = EDMAutil_configSyncAwithChaining(obj->edmaHandle,
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.satMonAddr,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        (uint8_t *)(SOC_translateAddress((uint32_t)obj->datapathCQ.rxSatData,SOC_TranslateAddr_Dir_TO_EDMA,NULL)),
        MMW_EDMA_CH_RX_SATURATION_MON, 
        MMW_EDMA_CH_RX_SATURATION_MON,
        false,
        MMW_EDMA_CH_RX_SATURATION_MON,
        aCnt,
        1,
        0,
        0,
        eventQueue,
        true, /*isFinalTransferInterruptEnabled */
        false,  /* isFinalChainingEnabled */
        MmwDemo_EDMA_CQTransferCompletionCallbackFxn,
        (uintptr_t) obj);
    if (retVal != EDMA_NO_ERROR)
    {
        return -1;
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *  @param[in] ptrDataPathObj Pointer to data path object.
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
int32_t MmwDemo_dataPathConfigCQ(MmwDemo_DataPathObj * ptrDataPathObj)
{
    ADCBuf_CQConf               cqConfig;
    rlRxSatMonConf_t*           ptrSatMonCfg;
    rlSigImgMonConf_t*          ptrSigImgMonCfg;
    MmwDemo_AnaMonitorCfg*      ptrAnaMonitorCfg;
    int32_t                     retVal;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &ptrDataPathObj->cliCommonCfg->anaMonCfg;

    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &ptrDataPathObj->cliCommonCfg->cqSatMonCfg[ptrDataPathObj->validProfileIdx];

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        retVal = mmwDemo_cfgRxSaturationMonitor(ptrSatMonCfg);
        if(retVal < 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d\n", retVal);

            MmwDemo_debugAssert(0);
        }
    }

    /* Config mmwaveLink to enable Signal Image band monitor - CQ1 */
    ptrSigImgMonCfg = &ptrDataPathObj->cliCommonCfg->cqSigImgMonCfg[ptrDataPathObj->validProfileIdx];

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        retVal = mmwDemo_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if(retVal < 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d\n", retVal);

            MmwDemo_debugAssert(0);
        }
    }

    /* Config CQ */
    if ((ptrAnaMonitorCfg->rxSatMonEn) || (ptrAnaMonitorCfg->sigImgMonEn))
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        cqConfig.cqDataWidth = 0; /* 16bit for mmw demo */

        /* CQ1 starts from the beginning of the buffer, address should be 16 bytes aligned */
        cqConfig.cq1AddrOffset = MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET;  
        cqConfig.cq2AddrOffset = MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;  /* address should be 16 bytes aligned. */

        retVal = ADCBuf_control(ptrDataPathObj->adcbufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            System_printf ("Error: Unable to configure CQ, errorCode[%d]\n", retVal);
            return -1;
        }
    }

    /* Save config pointer */
    ptrDataPathObj->datapathCQ.rxSatMonCfg = ptrSatMonCfg;
    ptrDataPathObj->datapathCQ.sigImgMonCfg = ptrSigImgMonCfg;
    ptrDataPathObj->datapathCQ.anaMonCfg= ptrAnaMonitorCfg;

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* Save CQ-Signal & Image band energy info in datapath object */
        ptrDataPathObj->datapathCQ.sigImgMonAddr = ADCBUF_MMWave_getCQBufAddr(ptrDataPathObj->adcbufHandle,
                                                                                   ADCBufMMWave_CQType_CQ1,
                                                                                   &retVal);
        MmwDemo_debugAssert (ptrDataPathObj->datapathCQ.sigImgMonAddr != NULL);

        /* This is for 16bit format in mmw demo */
        ptrDataPathObj->datapathCQ.sigImgMonTotalSize = (ptrSigImgMonCfg->numSlices + 1U) * 2U;

        /* Allocate data memory */
        ptrDataPathObj->datapathCQ.sigImgData = &gCQRxSigImgMemory;

    }
    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* Save CQ-Rx Saturation info in datapath object */
        ptrDataPathObj->datapathCQ.satMonAddr =
                    ADCBUF_MMWave_getCQBufAddr(ptrDataPathObj->adcbufHandle,
                                                   ADCBufMMWave_CQType_CQ2,
                                                   &retVal);
        MmwDemo_debugAssert    (ptrDataPathObj->datapathCQ.satMonAddr != NULL);

        /* This is for 16bit format in mmw demo */
        ptrDataPathObj->datapathCQ.satMonTotalSize = ptrSatMonCfg->numSlices + 1U;

        /* Allocate data memory */
        ptrDataPathObj->datapathCQ.rxSatData = &gCQRxSatMonMemory;
    }

    /* EDMA configuration for CQ */
    if ((ptrAnaMonitorCfg->rxSatMonEn) || (ptrAnaMonitorCfg->sigImgMonEn))
    {
        retVal = MmwDemo_dataPathConfigCQEdma(ptrDataPathObj);
        if(retVal < 0)
        {
            return -1;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      parses Profile, Chirp and Frame config and extracts parameters
 *      needed for processing chain configuration
 */
bool MmwDemo_parseProfileAndChirpConfig(MmwDemo_DataPathObj *dataPathObj)
{
    uint16_t    frameChirpStartIdx;
    uint16_t    frameChirpEndIdx;
    int16_t     frameTotalChirps;
    int32_t     errCode;
    uint32_t    profileLoopIdx, chirpLoopIdx;
    bool        foundValidProfile = false;
    uint16_t    channelTxEn = gMmwMCB.cfg.openCfg.chCfg.txChannelEn;
    uint8_t     channel;
    uint8_t     numRxAntennas = 0;
    uint8_t     rxAntOrder [SYS_COMMON_NUM_RX_CHANNEL];
    uint8_t     txAntOrder [SYS_COMMON_NUM_TX_ANTENNAS];
    int32_t     i;
    int32_t     txIdx, rxIdx;

    /* Find number of enabled channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        rxAntOrder[channel] = 0;
        if(gMmwMCB.cfg.openCfg.chCfg.rxChannelEn & (0x1<<channel))
        {
            rxAntOrder[numRxAntennas] = channel;
            /* Track the number of receive channels: */
            numRxAntennas++;
        }
    }
    dataPathObj->numRxAntennas = numRxAntennas;

    /* read frameCfg chirp start/stop*/
    frameChirpStartIdx = gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpStartIdx;
    frameChirpEndIdx = gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpEndIdx;
    frameTotalChirps = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */
    for (profileLoopIdx=0;
        ((profileLoopIdx<MMWAVE_MAX_PROFILE)&&(foundValidProfile==false));
        profileLoopIdx++)
    {
        uint32_t    mmWaveNumChirps = 0;
        bool        validProfileHasElevation=false;
        bool        validProfileHasOneTxPerChirp=false;
        uint16_t    validProfileTxEn = 0;
        uint16_t    validChirpTxEnBits[32]={0};
        MMWave_ProfileHandle profileHandle;

        profileHandle = gMmwMCB.cfg.ctrlCfg.u.frameCfg.profileHandle[profileLoopIdx];
        if (profileHandle == NULL)
            continue; /* skip this profile */

        /* get numChirps for this profile; skip error checking */
        MMWave_getNumChirps(profileHandle,&mmWaveNumChirps,&errCode);
        /* loop for chirps and find if it has valid chirps for the frame
           looping around for all chirps in a profile, in case
           there are duplicate chirps
         */
        for (chirpLoopIdx=1;chirpLoopIdx<=mmWaveNumChirps;chirpLoopIdx++)
        {
            MMWave_ChirpHandle chirpHandle;
            /* get handle and read ChirpCfg */
            if (MMWave_getChirpHandle(profileHandle,chirpLoopIdx,&chirpHandle,&errCode)==0)
            {
                rlChirpCfg_t chirpCfg;
                if (MMWave_getChirpCfg(chirpHandle,&chirpCfg,&errCode)==0)
                {
                    uint16_t chirpTxEn = chirpCfg.txEnable;
                    /* do chirps fall in range and has valid antenna enabled */
                    if ((chirpCfg.chirpStartIdx >= frameChirpStartIdx) &&
                        (chirpCfg.chirpEndIdx <= frameChirpEndIdx) &&
                        ((chirpTxEn & channelTxEn) > 0))
                    {
                        uint16_t idx = 0;
                        for (idx=(chirpCfg.chirpStartIdx-frameChirpStartIdx);idx<=(chirpCfg.chirpEndIdx-frameChirpStartIdx);idx++)
                        {
                            validChirpTxEnBits[idx] = chirpTxEn;
                            foundValidProfile = true;
                        }

                    }
                }
            }
        }
        /* now loop through unique chirps and check if we found all of the ones
           needed for the frame and then determine the azimuth/elevation antenna
           configuration
         */
        if (foundValidProfile) {
            int16_t nonElevFirstChirpIdx = -1;
            for (chirpLoopIdx=0;chirpLoopIdx<frameTotalChirps;chirpLoopIdx++)
            {
                bool validChirpHasElevation=false;
                bool validChirpHasOneTxPerChirp=false;
                uint16_t chirpTxEn = validChirpTxEnBits[chirpLoopIdx];
                if (chirpTxEn == 0) {
                    /* this profile doesnt have all the needed chirps */
                    foundValidProfile = false;
                    break;
                }
                /* check if this is an elevation TX chirp */
                validChirpHasElevation = (chirpTxEn==0x2);
                validProfileHasElevation |= validChirpHasElevation;
                /* if not, then check the MIMO config */
                if (!validChirpHasElevation)
                {
                    validChirpHasOneTxPerChirp = ((chirpTxEn==0x1) || (chirpTxEn==0x4));
                    /* if this is the first chirp without elevation, record the chirp's
                       MIMO config as profile's MIMO config. We dont handle intermix
                       at this point */
                    if (nonElevFirstChirpIdx==-1) {
                        validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
                        nonElevFirstChirpIdx = chirpLoopIdx;
                    }
                    /* check the chirp's MIMO config against Profile's MIMO config */
                    if (validChirpHasOneTxPerChirp != validProfileHasOneTxPerChirp)
                    {
                        /* this profile doesnt have all chirps with same MIMO config */
                        foundValidProfile = false;
                        break;
                    }
                }
                /* save the antennas actually enabled in this profile */
                validProfileTxEn |= chirpTxEn;
            }
        }

        /* found valid chirps for the frame; mark this profile valid */
        if (foundValidProfile==true) {
            rlProfileCfg_t  profileCfg;
            uint32_t        numTxAntAzim = 0;
            uint32_t        numTxAntElev = 0;

            dataPathObj->validProfileIdx = profileLoopIdx;
            dataPathObj->numTxAntennas = 0;
            if (validProfileHasElevation)
            {
                numTxAntElev = 1;
            }
            if (!validProfileHasOneTxPerChirp)
            {
                numTxAntAzim=1;
            }
            else
            {
                if (validProfileTxEn & 0x1)
                {
                    numTxAntAzim++;
                }
                if (validProfileTxEn & 0x4)
                {
                    numTxAntAzim++;
                }
            }
            /*System_printf("Azimuth Tx: %d (MIMO:%d), Elev Tx:%d\n",
                            numTxAntAzim,validProfileHasMIMO,numTxAntElev);*/
            dataPathObj->numTxAntennas = numTxAntAzim + numTxAntElev;
            dataPathObj->numVirtualAntAzim = numTxAntAzim * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntElev = numTxAntElev * dataPathObj->numRxAntennas;
            dataPathObj->numVirtualAntennas = dataPathObj->numVirtualAntAzim + dataPathObj->numVirtualAntElev;

            /* Sanity Check: Ensure that the number of antennas is within system limits */
            MmwDemo_debugAssert (dataPathObj->numVirtualAntennas > 0);
            MmwDemo_debugAssert (dataPathObj->numVirtualAntennas <= (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL));

            /* Copy the Rx channel compensation coefficients from common area to data path structure */
            if (validProfileHasOneTxPerChirp)
            {
                for (i = 0; i < dataPathObj->numTxAntennas; i++)
                {
                    txAntOrder[i] = log2Approx(validChirpTxEnBits[i]);
                }
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] =
                                dataPathObj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[txAntOrder[txIdx]*SYS_COMMON_NUM_RX_CHANNEL + rxAntOrder[rxIdx]];

                    }

                }
            }
            else
            {
                cmplx16ImRe_t one;
                one.imag = 0;
                one.real = 0x7fff;
                for (txIdx = 0; txIdx < dataPathObj->numTxAntennas; txIdx++)
                {
                    for (rxIdx = 0; rxIdx < dataPathObj->numRxAntennas; rxIdx++)
                    {
                        dataPathObj->compRxChanCfg.rxChPhaseComp[txIdx*dataPathObj->numRxAntennas + rxIdx] = one;
                    }

                }
            }

            /* Get the profile configuration: */
            if (MMWave_getProfileCfg(profileHandle,&profileCfg, &errCode) < 0)
            {
                MmwDemo_debugAssert(0);
                return false;
            }

#ifndef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
            /* Check frequency slope */
            if (profileCfg.freqSlopeConst < 0)
            {
                System_printf("Frequency slope must be positive\n");
                MmwDemo_debugAssert(0);
            }
#endif

            dataPathObj->numAdcSamples = profileCfg.numAdcSamples;
            dataPathObj->numRangeBins = MmwDemo_pow2roundup(dataPathObj->numAdcSamples);
            dataPathObj->numChirpsPerFrame = frameTotalChirps *
                                              gMmwMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numLoops;

            dataPathObj->numAngleBins = MMW_NUM_ANGLE_BINS;
            dataPathObj->numDopplerBins = dataPathObj->numChirpsPerFrame/dataPathObj->numTxAntennas;
            dataPathObj->numRangeBinsPerTransfer = MMW_NUM_RANGE_BINS_PER_TRANSFER;
            dataPathObj->rangeResolution = MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC * profileCfg.digOutSampleRate * 1e3 /
                    (2 * profileCfg.freqSlopeConst * ((3.6*1e3*900)/(1U << 26)) * 1e12 * dataPathObj->numRangeBins);

            dataPathObj->xyzOutputQFormat    = (uint32_t) ceil(log10(16./fabs(dataPathObj->rangeResolution))/log10(2));
            dataPathObj->dataPathMode = DATA_PATH_WITH_ADCBUF;
            dataPathObj->frameStartIntCounter = 0;
            dataPathObj->interFrameProcToken = 0;
        }
    }
    return foundValidProfile;
}

void MmwDemo_measurementResultOutput(MmwDemo_DataPathObj *obj)
{
    /* Send the received DSS calibration info through CLI */
    CLI_write ("compRangeBiasAndRxChanPhase");
    CLI_write (" %.7f", obj->cliCommonCfg->compRxChanCfg.rangeBias);
    int32_t i;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        CLI_write (" %.5f", (float) obj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[i].real/32768.);
        CLI_write (" %.5f", (float) obj->cliCommonCfg->compRxChanCfg.rxChPhaseComp[i].imag/32768.);
    }
    CLI_write ("\n");

}

/** @brief Transmits detection data over UART
*
*    The following data is transmitted:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and icluding the number of TLV items
*    TLV Items:
*    2. If detectedObjects flag is set, pbjOut structure containing range,
*       Doppler, and X,Y,Z location for detected objects,
*       size = sizeof(objOut_t) * number of detected objects
*    3. If logMagRange flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint16_t)
*    4. If noiseProfile flag is set,  noiseProfile,
*       size = number of range bins * sizeof(uint16_t)
*    7. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
*       range cubed matrix, size = number of Rx Azimuth virtual antennas *
*       number of chirps per frame * sizeof(uint32_t)
*    8. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
*       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
*    9. If statsInfo flag is set, the stats information
*   @param[in] uartHandle   UART driver handle
*   @param[in] obj          Pointer data path object MmwDemo_DataPathObj
*/

void MmwDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                    MmwDemo_DataPathObj *obj)
{
    MmwDemo_output_message_header header;
    MmwDemo_GuiMonSel   *pGuiMonSel;
    uint32_t tlvIdx = 0;
    uint32_t i;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];

    MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];

    /* Get Gui Monitor configuration */
    pGuiMonSel = &gMmwMCB.cliCfg.guiMonSel;

    /* Clear message header */
    memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));
    /* Header: */
    header.platform = 0xA1443;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.numDetectedObj = obj->numObjOut;
    header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                        (MMWAVE_SDK_VERSION_MINOR << 16) |
                        (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(MmwDemo_output_message_header);
    if (pGuiMonSel->detectedObjects && (obj->numObjOut > 0))
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        tl[tlvIdx].length = sizeof(MmwDemo_detectedObj) * obj->numObjOut +
                            sizeof(MmwDemo_output_message_dataObjDescr);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->logMagRange)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * obj->numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->noiseProfile)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_NOISE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * obj->numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP;
        tl[tlvIdx].length = obj->numRangeBins * obj->numVirtualAntAzim * sizeof(uint32_t);
        packetLen += sizeof(MmwDemo_output_message_tl) +  tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeDopplerHeatMap)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
        tl[tlvIdx].length = obj->numRangeBins * obj->numDopplerBins * sizeof(uint16_t);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->statsInfo)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    header.numTLVs = tlvIdx;
    /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles =  Pmu_getCount(0);
    header.frameNumber = obj->frameStartIntCounter;


    UART_writePolling (uartHandle,
                       (uint8_t*)&header,
                       sizeof(MmwDemo_output_message_header));

    tlvIdx = 0;
    /* Send detected Objects */
    if ((pGuiMonSel->detectedObjects == 1) && (obj->numObjOut > 0))
    {
        MmwDemo_output_message_dataObjDescr descr;

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        /* Send objects descriptor */
        descr.numDetetedObj = (uint16_t) obj->numObjOut;
        descr.xyzQFormat = (uint16_t) obj->xyzOutputQFormat;
        UART_writePolling (uartHandle, (uint8_t*)&descr, sizeof(MmwDemo_output_message_dataObjDescr));

        /*Send array of objects */
        UART_writePolling (uartHandle, (uint8_t*)obj->objOut, sizeof(MmwDemo_detectedObj) * obj->numObjOut);
        tlvIdx++;
    }

    /* Send Range profile */
    if (pGuiMonSel->logMagRange)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        for(i = 0; i < obj->numRangeBins; i++)
        {
            UART_writePolling (uartHandle,
                    (uint8_t*)&obj->rangeDopplerLogMagMatrix[i*obj->numDopplerBins],
                    sizeof(uint16_t));
        }
        tlvIdx++;
    }

    /* Send noise profile */
    if (pGuiMonSel->noiseProfile)
    {
        uint32_t maxDopIdx = obj->numDopplerBins/2 -1;
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        for(i = 0; i < obj->numRangeBins; i++)
        {
            UART_writePolling (uartHandle,
                    (uint8_t*)&obj->rangeDopplerLogMagMatrix[i*obj->numDopplerBins + maxDopIdx],
                    sizeof(uint16_t));
        }
        tlvIdx++;
    }

    /* Send data for static azimuth heatmap */
    if (pGuiMonSel->rangeAzimuthHeatMap)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        UART_writePolling (uartHandle,
                (uint8_t *) obj->azimuthStaticHeatMap,
                obj->numRangeBins * obj->numVirtualAntAzim * sizeof(uint32_t));

        tlvIdx++;
    }

    /* Send data for range/Doppler heatmap */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        UART_writePolling (uartHandle,
                (uint8_t*)obj->rangeDopplerLogMagMatrix,
                tl[tlvIdx].length);
        tlvIdx++;
    }

    /* Send stats information */
    if (pGuiMonSel->statsInfo == 1)
    {
        MmwDemo_output_message_stats stats;
        stats.interChirpProcessingMargin = 0; /* Not applicable */
        stats.interFrameProcessingMargin = (uint32_t) (obj->timingInfo.interFrameProcessingEndMargin/R4F_CLOCK_MHZ); /* In micro seconds */
        stats.interFrameProcessingTime = (uint32_t) (obj->timingInfo.interFrameProcCycles/R4F_CLOCK_MHZ); /* In micro seconds */
        stats.transmitOutputTime = (uint32_t) (obj->timingInfo.transmitOutputCycles/R4F_CLOCK_MHZ); /* In micro seconds */
        stats.activeFrameCPULoad = obj->timingInfo.activeFrameCPULoad;
        stats.interFrameCPULoad = obj->timingInfo.interFrameCPULoad;

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        UART_writePolling (uartHandle,
                           (uint8_t*)&stats,
                           tl[tlvIdx].length);
        tlvIdx++;
    }

    /* Send padding bytes */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling (uartHandle,
                            (uint8_t*)padding,
                            numPaddingBytes);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to trigger the Front end to start generating chirps.
 *
 *  @retval
 *      Not Applicable.
 */
int32_t MmwDemo_dataPathStart (void)
{
    MMWave_CalibrationCfg   calibrationCfg;
    int32_t                 errCode = 0;
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;

    dataPathObj->frameStartIntCounter = 0;
    dataPathObj->interFrameProcToken = 0;
    
    gMmwMCB.lvdsStream.hwFrameDoneCount = 0;
    gMmwMCB.lvdsStream.swFrameDoneCount = 0;

    /* Initialize the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start (gMmwMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        //System_printf ("Error: mmWave Control Start failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
    }
    else
    {
        /* Update data path stop status */
        dataPathObj->datapathStopped = false;
    }

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      Configures LVDS/CBUFF HW session for transmitting ADC data
 *
 *  @param[in]  dataPathObj
 *      Pointer to datapath object
 *  @retval
 *      Not Applicable.
 */
/*Unverified code. Conflicts with data path processing and should not be used.*/
void MmwDemo_configLVDSHwData (MmwDemo_DataPathObj *dataPathObj)
{
    int32_t errCode;
    
    /* Delete previous CBUFF HW session if one was configured */
    if(gMmwMCB.lvdsStream.hwSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteHwSession(gMmwMCB.lvdsStream.hwSessionHandle);
    }
    
    /* Configure HW session */    
    if (MmwDemo_LVDSStreamHwConfig(dataPathObj) < 0)
    {
        //System_printf("Failed LVDS stream HW configuration\n");
        MmwDemo_debugAssert(0);
        return;
    }
    
    /* If HW LVDS stream is enabled, start the session here so that ADC samples will be 
    streamed out as soon as the first chirp samples land on ADC*/
    if(CBUFF_activateSession (gMmwMCB.lvdsStream.hwSessionHandle, &errCode) < 0)
    {
        //System_printf("Failed to activate CBUFF session for LVDS stream HW. errCode=%d\n",errCode);
        MmwDemo_debugAssert(0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathConfig (void)
{
    int32_t    retVal = 0;
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;

    /* Configure ADCBuf Config and get the valid number of RX antennas
       do this first as we need the numRxAntennas in MmwDemo_parseProfileAndChirpConfig
       to get the Virtual Antennas */
    /* Parse the profile and chirp configs and get the valid number of TX Antennas */
    if (MmwDemo_parseProfileAndChirpConfig(dataPathObj) == true)
    {

        retVal = mmwDemo_cfgAnalogMonitor(&dataPathObj->cliCommonCfg->anaMonCfg);
        if (retVal != 0)
        {
            System_printf ("Error: rlRfAnaMonConfig returns error = %d\n", retVal);

            MmwDemo_debugAssert(0);
        }

        if (MmwDemo_ADCBufConfig(dataPathObj) < 0)
        {
            //System_printf("Error: ADCBuf config failed \n");
            MmwDemo_debugAssert (0);
        }

        /* Configure CQ */
        MmwDemo_dataPathConfigCQ(dataPathObj);

        /* Now we are ready to allocate and config the data buffers */
        MmwDemo_dataPathCfgBuffers(dataPathObj, &gMmwL3heap);
        /* Configure one-time EDMA and HWA parameters */
        MmwDemo_dataPathConfigCommon(dataPathObj);

        /*Delete all valid LVDS stream sessions*/
        /*Unverified code. Conflicts with data path processing and should not be used.*/
        MmwDemo_LVDSStreamDelete();

        /* Configure LVDS transfer of ADC data */
        /*Unverified code. Conflicts with data path processing and should not be used.*/
        if(dataPathObj->cliCfg->lvdsStreamCfg.dataFmt != 0)
        {
            MmwDemo_configLVDSHwData(dataPathObj);
        }              
        
        /* Config HWA for 1D processing and keep it ready for immediate processingh
           as soon as Front End starts generating chirps */
        MmwDemo_config1D_HWA(dataPathObj);
        MmwDemo_dataPathTrigger1D(dataPathObj);
    }
    else
    {
        /* no valid profile found - assert! */
        MmwDemo_debugAssert(0);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *  This function is called at the init time from @ref MmwDemo_initTask.
 *  It initializes drivers: ADCBUF, HWA, EDMA, and semaphores used
 *  by  @ref MmwDemo_dataPathTask
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathInit(MmwDemo_DataPathObj *obj)
{
    MmwDemo_dataPathObjInit(obj, &gMmwMCB.cliCfg, &gMmwMCB.cliCommonCfg);

    /* Initialize the ADCBUF */
    ADCBuf_init();

    /* Initialize HWA */
    MmwDemo_hwaInit(obj);

    /* Initialize EDMA */
    MmwDemo_edmaInit(obj);
}

void MmwDemo_dataPathOpen(MmwDemo_DataPathObj *obj)
{
    /*****************************************************************************
     * Start HWA, EDMA and ADCBUF drivers:
     *****************************************************************************/
    MmwDemo_hwaOpen(obj, gMmwMCB.socHandle);
    MmwDemo_edmaOpen(obj);
    MmwDemo_ADCBufOpen(obj);
}

/**
 *  @b Description
 *  @n
 *      The function is used to Stop data path.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathStop (MmwDemo_DataPathObj *obj)
{
    obj->datapathStopped = true;
    if(obj->interFrameProcToken == 0)
    {
        MmwDemo_notifyDathPathStop();
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMCB.ctrlHandle, &errCode) < 0)
        {
            //System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert (0);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function to mmwave which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0
 */
int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitomplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitomplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

                    /* Display the calibration status: */
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    gMmwMCB.stats.frameTriggerReady++;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    gMmwMCB.stats.failedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    gMmwMCB.stats.calibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gMmwMCB.stats.sensorStopped++;

                    /*Received Frame Stop async event from BSS. Post event to sensor management task.*/
                    MmwDemo_notifyBssSensorStop();
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Transmit user data over LVDS interface.
 *
 *  @param[in]  dataPathObj
 *      Pointer to datapath object
 *
 */
/*Unverified code. Conflicts with data path processing and should not be used.*/
void MmwDemo_transferLVDSUserData(MmwDemo_DataPathObj *dataPathObj)
{
    int32_t errCode;
    
    /* Delete previous SW session if it exists. SW session is being
       reconfigured every frame because number of detected objects
       may change from frame to frame which implies that the size of
       the streamed data may change. */        
    if(gMmwMCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession(gMmwMCB.lvdsStream.swSessionHandle);
    }
    
    /* Configure SW session for this subframe */
    if (MmwDemo_LVDSStreamSwConfig(dataPathObj) < 0)
    {
        //System_printf("Failed LVDS stream SW configuration\n");
        MmwDemo_debugAssert(0);
        return;
    }
    
    /* Populate user data header that will be streamed out*/
    gMmwMCB.lvdsStream.userDataHeader.frameNum  = dataPathObj->frameStartIntCounter;
    gMmwMCB.lvdsStream.userDataHeader.detObjNum = dataPathObj->numObjOut;
    gMmwMCB.lvdsStream.userDataHeader.reserved  = 0xABCD;
    
    /* If SW LVDS stream is enabled, start the session here. User data will imediatelly
       start to stream over LVDS.*/
    if(CBUFF_activateSession (gMmwMCB.lvdsStream.swSessionHandle, &errCode) < 0)
    {
        //System_printf("Failed to activate CBUFF session for LVDS stream SW. errCode=%d\n",errCode);
        MmwDemo_debugAssert(0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used for data path processing and to transmit the
 *      detected objects through the UART output port.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dataPathTask(UArg arg0, UArg arg1)
{
    MmwDemo_DataPathObj *dataPathObj = &gMmwMCB.dataPathObj;
    uint16_t numDetectedObjects;
    uint32_t startTime, transmitOutStartTime;
    uint32_t txOrder[SYS_COMMON_NUM_TX_ANTENNAS] = {0,2,1};
    while(1)
    {
        Semaphore_pend(dataPathObj->frameStart_semHandle, BIOS_WAIT_FOREVER);

        Load_update();
        dataPathObj->timingInfo.interFrameCPULoad=Load_getCPULoad();

        MmwDemo_dataPathWait1D(dataPathObj);
        /* 1st Dimension FFT done! */

        Load_update();
        dataPathObj->timingInfo.activeFrameCPULoad=Load_getCPULoad();

        startTime = Pmu_getCount(0);

        if(dataPathObj->cliCfg->calibDcRangeSigCfg.enabled)
        {
             if (dataPathObj->cliCfg->calibDcRangeSigCfg.numAvgChirps <  dataPathObj->numDopplerBins)
             {
                 dataPathObj->cliCfg->calibDcRangeSigCfg.enabled = 0;
                 dataPathObj->dcRangeForcedDisableCntr++;
             }
             else
             {
                 MmwDemo_dcRangeSignatureCompensation(dataPathObj);
             }
        }

        /* Clutter removal */
        if (dataPathObj->cliCfg->clutterRemovalCfg.enabled)
        {
            int32_t rngIdx, antIdx, dopIdx;
            cmplx16ImRe_t *fftOut1D = (cmplx16ImRe_t *) dataPathObj->radarCube;
            cmplx32ImRe_t meanVal;

            for (rngIdx = 0; rngIdx < dataPathObj->numRangeBins; rngIdx++)
            {
                for (antIdx = 0; antIdx < dataPathObj->numVirtualAntennas; antIdx++)
                {
                    meanVal.real = 0;
                    meanVal.imag = 0;
                    for (dopIdx = 0; dopIdx < dataPathObj->numDopplerBins; dopIdx++)
                    {
                        meanVal.real += fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) +
                                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].real;
                        meanVal.imag += fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) +
                                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].imag;
                    }
                    meanVal.real = meanVal.real/dataPathObj->numDopplerBins;
                    meanVal.imag = meanVal.imag/dataPathObj->numDopplerBins;
                    for (dopIdx = 0; dopIdx < dataPathObj->numDopplerBins; dopIdx++)
                    {
                        fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) +
                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].real -= meanVal.real;
                        fftOut1D[rngIdx*(dataPathObj->numDopplerBins*dataPathObj->numVirtualAntennas) +
                                 antIdx + dopIdx*(dataPathObj->numVirtualAntennas)].imag -= meanVal.imag;
                    }
                }
            }
        }

        MmwDemo_process2D(dataPathObj);
        /* 2nd Dimension FFT done! */

        /* Procedure for range bias measurement and Rx channels gain/phase offset measurement */
        if(dataPathObj->cliCommonCfg->measureRxChanCfg.enabled)
        {
            MmwDemo_rangeBiasRxChPhaseMeasure(
                    dataPathObj->cliCommonCfg->measureRxChanCfg.targetDistance,
                    dataPathObj->rangeResolution,
                    dataPathObj->cliCommonCfg->measureRxChanCfg.searchWinSize,
                    dataPathObj->rangeDopplerLogMagMatrix,
                    dataPathObj->numDopplerBins,
                    dataPathObj->numVirtualAntennas,
                    dataPathObj->numVirtualAntennas * dataPathObj->numDopplerBins,
                    dataPathObj->radarCube,
                    dataPathObj->numRxAntennas,
                    dataPathObj->numTxAntennas,
                    txOrder,
                    &dataPathObj->cliCommonCfg->compRxChanCfg);

        }

        MmwDemo_processCfar(dataPathObj, &numDetectedObjects);
        /* CFAR done! */

        /* Postprocessing/angle estimation */
        dataPathObj->numHwaCfarDetections = numDetectedObjects;
        MmwDemo_processAngle(dataPathObj);

        /* Calculate noise floor. This is for EVM diagnostics only. Assumes a stationary scene */
        dataPathObj->noiseEnergy = calcNoiseFloor (dataPathObj->radarCube, dataPathObj->numDopplerBins,
                dataPathObj->numRangeBins, dataPathObj->numVirtualAntennas);

        transmitOutStartTime = Pmu_getCount(0);

        /* Sending range bias and Rx channel phase offset measurements to MSS and from there to CLI */
         if(dataPathObj->cliCommonCfg->measureRxChanCfg.enabled)
         {
             MmwDemo_measurementResultOutput (dataPathObj);
         }

        /* Send user data over LVDS */
        /*Unverified code. Conflicts with data path processing and should not be used.*/
        if(dataPathObj->cliCfg->lvdsStreamCfg.isSwEnabled == 1)
        {
            MmwDemo_transferLVDSUserData(dataPathObj);
        }    
        
        MmwDemo_transmitProcessedOutput(gMmwMCB.loggingUartHandle,
                                        dataPathObj);

        dataPathObj->timingInfo.transmitOutputCycles = Pmu_getCount(0) - transmitOutStartTime;

        /* Processing cycles for 2D, CFAR, Azimuth/Elevation
           processing excluding sending out data */
        dataPathObj->timingInfo.interFrameProcessingEndTime = Pmu_getCount(0);
        dataPathObj->timingInfo.interFrameProcCycles = dataPathObj->timingInfo.interFrameProcessingEndTime - startTime -
            dataPathObj->timingInfo.transmitOutputCycles;

        dataPathObj->interFrameProcToken--;
        if(dataPathObj->datapathStopped == true)
        {
            MmwDemo_notifyDathPathStop();
        }
        else
        {
            /* Prepare for next frame */
            MmwDemo_config1D_HWA(dataPathObj);
            MmwDemo_dataPathTrigger1D(dataPathObj);
        }
    }
}


/**
 *  @b Description
 *  @n
 *      Frame start interrupt handler
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_frameStartIntHandler(uintptr_t arg)
{
    MmwDemo_DataPathObj * dpObj = &gMmwMCB.dataPathObj;

    /* Increment interrupt counter for debugging purpose */
    dpObj->frameStartIntCounter++;

    /* Note: this is valid after the first frame */
    dpObj->timingInfo.interFrameProcessingEndMargin =
            Pmu_getCount(0) - dpObj->timingInfo.interFrameProcessingEndTime;

    /* Check if previous chirp processing has completed */
    MmwDemo_debugAssert(dpObj->interFrameProcToken == 0);
    dpObj->interFrameProcToken++;

    Semaphore_post(dpObj->frameStart_semHandle);

}


/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_initTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;

    /* Debug Message: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/

    /* Initialize the UART */
    UART_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Initialize the GPIO */
    GPIO_init ();

    /* Initialize the Data Path: */
    MmwDemo_dataPathInit(&gMmwMCB.dataPathObj);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN6_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN6_PADBE, SOC_XWR14XX_PINN6_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR14XX_PINN5_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR14XX_PINN5_PADBD, SOC_XWR14XX_PINN5_PADBD_MSS_UARTA_RX);

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone   = 1;

    /* Open the UART Instance */
    gMmwMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMCB.commandUartHandle == NULL)
    {
        //System_printf("Error: Unable to open the Command UART Instance\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.commandUartHandle);

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 0;

    /* Open the Logging UART Instance: */
    gMmwMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMCB.loggingUartHandle == NULL)
    {
        //System_printf("Error: Unable to open the Logging UART Instance\n");
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf("Debug: UART Instance %p has been opened successfully\n", gMmwMCB.loggingUartHandle);

    /* Initialize LVDS streaming components */
    /*Unverified code. Conflicts with data path processing and should not be used.*/
    if ((errCode = MmwDemo_LVDSStreamInit()) < 0 )
    {
        //System_printf ("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration: */
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gMmwMCB.socHandle;
    initCfg.eventFxn                = MmwDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;

    /* Initialize and setup the mmWave Control module */
    gMmwMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        //System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf ("Debug: mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    if (MMWave_sync (gMmwMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to synchronize the mmWave control module */
        //System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    //System_printf ("Debug: mmWave Control Synchronization was successful\n");

    MmwDemo_dataPathOpen(&gMmwMCB.dataPathObj);

    /* Configure banchmark counter */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 5;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialize the CLI Module:
     *****************************************************************************/
    MmwDemo_CLIInit();

    /*****************************************************************************
     * Initialize the Sensor Management Module:
     *****************************************************************************/
    if (MmwDemo_sensorMgmtInit() < 0)
        return;

    /* Register Frame start interrupt handler */
    {
        SOC_SysIntListenerCfg  socIntCfg;
        int32_t errCode;

        Semaphore_Params       semParams;

        /* Register frame start interrupt listener */
        socIntCfg.systemInterrupt  = SOC_XWR14XX_DSS_FRAME_START_IRQ;
        socIntCfg.listenerFxn      = MmwDemo_frameStartIntHandler;
        socIntCfg.arg              = (uintptr_t)NULL;
        if (SOC_registerSysIntListener(gMmwMCB.socHandle, &socIntCfg, &errCode) == NULL)
        {
            //System_printf("Error: Unable to register frame start interrupt listener , error = %d\n", errCode);
            MmwDemo_debugAssert (0);
            return;
        }

        Semaphore_Params_init(&semParams);
        semParams.mode = Semaphore_Mode_BINARY;
        gMmwMCB.dataPathObj.frameStart_semHandle = Semaphore_create(0, &semParams, NULL);
    }

    /*****************************************************************************
     * Launch the Main task
     * - The main demo task
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = 4;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_dataPathTask, &taskParams, NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction.
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        //System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return -1;
    }

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMCB, 0, sizeof(MmwDemo_MCB));

    gMmwMCB.socHandle = socHandle;

    /* Initialize the DEMO configuration: */
    gMmwMCB.cfg.sysClockFrequency = (200 * 1000000);
    gMmwMCB.cfg.loggingBaudRate   = 921600;
    gMmwMCB.cfg.commandBaudRate   = 115200;

#if 0
    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the Millimeter Wave Demo\n");
    System_printf ("**********************************************\n");
#endif

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    Task_create(MmwDemo_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}


