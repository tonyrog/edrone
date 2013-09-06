# Hardware inventory AR Drone 2.0

## GPIO


OMAP PINS

| Pin | Direction | Comment |
|-----|-----------|---------|
| 171 | OUTPUT    | kill_uart_m1 |
| 172 | OUTPUT    | kill_uart_m2 |
| 173 | OUTPUT    | kill_uart_m3 |
| 174 | OUTPUT    | kill_uart_m4 |
| 175 | OUTPUT    | motor_enable |
| 176 | INPUT     | cut-out |
| 177 | OUTPUT    | PIC reset |
| 178 | INPUT     | PIC ICSP Data |
| 179 | OUTPUT    | PIC ICSP Clock |
| 180 | OUTPUT    | MB green LED |
| 181 | OUTPUT    | MB Red LED |
| 87  | INPUT     | Reset Button |
| 89  | OUTPUT    | 5V cut-out |

Init

	89  = 1        Disabling 5V power supply
    177 = 1        PIC reset disable
    181 = 1        Red LED on
    180 = 1        Green LED on

MMC PINS (wifi ar6103 on MMC2)

| Pin | Direction | Comment |
|-----|-----------|---------|
|  85 |  OUTPUT   | GPIO\_ATH\_WARM\_RST
|  86 |  OUTPUT   | GPIO\_ATH\_RST_WLAN

| Pin | Direction | Comment |
|-----|-----------|---------|
|  51 |  OUTPUT   | USB HOST mode = 0
|  76 |   xxx     | GPIO\_MMC1\_CD

| Pin | Direction | Comment |
|-----|-----------|---------|
| 78  |  INPUT    | rev(0)  |
| 79  |  INPUT    | rev(1)  |
| 80  |  INPUT    | rev(2)  |
| 81  |  INPUT    | rev(3)  |

    Revision = (get(81)<<3)|(get(80)<<2)|(get(79)<<1)|get(78)
    91   OUTPUT ?
    92   OUTPUT ?
    93   OUTPUT ?


## Driver loaded

    ov7670
    soc1040
    omap3_isp
    ar6000

## version

    MB HW0 - Mykonos 2 Proto A
    MB HW1 - Mykonos 2 Proto B
    MB HW2 - Mykonos 2 DV1/2
    MB HW3 - Mykonos 2 DV3
    MB HW4 - Mykonos 2 PV1
    MB HW5 - Mykonos 2 MassProd
    MB HW6 - Mykonos 2 MassProd USB

## cpuinfo

    Processor       : ARMv7 Processor rev 2 (v7l)
    BogoMIPS        : 996.74
    Features        : swp half thumb fastmult vfp edsp neon vfpv3 
    CPU implementer : 0x41
    CPU architecture: 7
    CPU variant     : 0x3
    CPU part        : 0xc08
    CPU revision    : 2
    Hardware        : mykonos2 board
    Revision        : 0006
    Serial          : 0000000000000000

## uarts used

    /dev/tty00   edrone_motorboard.erl
    /dev/ttyO1   edrone_navboard.erl
    /dev/ttyPA0
    /dev/ttyPA1
    /dev/ttyPA2

## config strings

### general
    general:gps_soft
    general:num_version_config
    general:num_version_mb
    general:num_version_soft
    general:drone_serial
    general:soft_build_date
    general:motor1_soft
    general:motor1_hard
    general:motor1_supplier
    general:motor2_soft
    general:motor2_hard
    general:motor2_supplier
    general:motor3_soft
    general:motor3_hard
    general:motor3_supplier
    general:motor4_soft
    general:motor4_hard
    general:motor4_supplier
    general:ardrone_name
    general:flying_time
    general:navdata_demo
    general:navdata_options
    general:com_watchdog
    general:video_enable
    general:vision_enable
    general:vbat_min
    general:localtime
    general:gps_hard
    general:localtime_zone
    general:timezone
    general:battery_type

### control

    control:accs_offset
    control:accs_gains
    control:gyros_offset
    control:gyros_gains
    control:gyros110_offset
    control:gyros110_gains
    control:magneto_offset
    control:magneto_radius
    control:gyro_offset_thr_x
    control:gyro_offset_thr_y
    control:gyro_offset_thr_z
    control:pwm_ref_gyros
    control:osctun_value
    control:osctun_test
    control:control_level
    control:euler_angle_max
    control:altitude_max
    control:altitude_min
    control:control_iphone_tilt
    control:control_vz_max
    control:control_yaw
    control:outdoor
    control:flight_without_shell
    control:autonomous_flight
    control:manual_trim
    control:indoor_euler_angle_max
    control:indoor_control_vz_max
    control:indoor_control_yaw
    control:outdoor_euler_angle_max
    control:outdoor_control_vz_max
    control:outdoor_control_yaw
    control:flying_mode
    control:hovering_range
    control:flight_anim
    control:flying_camera_mode
    control:flying_camera_enable

leds

    leds:leds_anim

video

    video:camif_fps
    video:codec_fps
    video:camif_buffers
    video:num_trackers
    video:video_codec
    video:video_slices
    video:video_live_socket
    video:video_storage_space
    video:bitrate
    video:max_bitrate
    video:bitrate_ctrl_mode
    video:bitrate_storage
    video:video_channel
    video:video_on_usb
    video:video_file_index
    video:exposure_mode
    video:saturation_mode
    video:whitebalance_mode


