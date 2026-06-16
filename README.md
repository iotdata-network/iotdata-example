# iotdata examples

Working examples demonstrating iotdata encoding, decoding, simulation, and radio
transport across Linux and ESP32 platforms.

All examples share the common variant suite and mesh protocol definitions in the
`iotdata/` directory. The EBYTE E22-900T22U LoRa radio driver is an external
dependency required by both the ESP32 transmitter and the Linux gateway —
install from
[github.com/matthewgream/e22900t22u](https://github.com/matthewgream/e22900t22u).

## iotdata/ — Variant Suite and Mesh Protocol

Shared definitions used by all examples.

**`iotdata_variant_suite.h`** defines 9 sensor variants representing common
real-world sensor configurations:

| Variant | Name                | Description                                                                           |
| ------- | ------------------- | ------------------------------------------------------------------------------------- |
| 0       | `weather_station`   | Full outdoor weather station (environment, wind, rain, solar, clouds, AQI, radiation) |
| 1       | `air_quality`       | Indoor/outdoor air quality monitor (SEN55/SEN66-style: PM, VOC, NOx, CO₂)             |
| 2       | `soil_moisture`     | Agricultural soil probe (soil temperature, moisture %, burial depth)                  |
| 3       | `water_level`       | River/tank level gauge (water temperature, level)                                     |
| 4       | `snow_depth`        | Ultrasonic snow depth sensor with solar power monitoring                              |
| 5       | `environment`       | Simple T/H/P sensor (BME280-style)                                                    |
| 6       | `wind_station`      | Standalone anemometer with solar                                                      |
| 7       | `rain_gauge`        | Standalone rain collector with temperature for freeze detection                       |
| 8       | `radiation_monitor` | Geiger counter station with environmental context                                     |

Every variant includes battery and link quality in the first two slots for
consistent telemetry. Variants use 1 or 2 presence bytes depending on the number
of fields.

**`iotdata_mesh.h`** implements a mesh relay protocol on variant 15 (reserved).
Hop nodes forward sensor packets toward a gateway without the sensors needing
any mesh awareness. The protocol includes beacons (route advertisement),
forwards (relayed sensor data with TTL), ACKs, route errors, neighbour reports,
and ping/pong. A duplicate suppression ring buffer prevents reprocessing of
already-seen packets.

## simulator/ — Standalone Simulator

A Linux command-line tool that exercises the full variant suite without any
hardware. It instantiates multiple sensors across all 9 variant types, each
producing realistic readings with random-walk drift, battery drain, and
staggered transmission intervals.

Packets are encoded using iotdata, then immediately decoded and printed with
full field detail — useful for verifying encode/decode round-trips and
inspecting the binary wire format.

Build and run:

```bash
cd simulator
make
./simulator [seed] [packet_count]
```

The simulator uses a deterministic xorshift32 RNG, so a given seed produces
repeatable output.

```text
# ./simulator
=== Simulator: 16 sensors, seed=12345 ===

  ID  Variant             Station
  --  ------------------  -------
   0  weather_station     1
   1  air_quality         2
   2  soil_moisture       3
   3  environment         4
   4  wind_station        5
   5  soil_moisture       6
   6  air_quality         7
   7  weather_station     8
   8  soil_moisture       9
   9  water_level         10
  10  water_level         11
  11  wind_station        12
  12  snow_depth          13
  13  soil_moisture       14
  14  rain_gauge          15
  15  radiation_monitor   16

[    0.4s] #1    stn=  3 soil_moisture      seq=0   bytes=11 [200300003FC16752A08804]:   bat=77%  rssi=-100 snr=0  T=18.50  D=34  H=42%  F=1
[    0.8s] #2    stn= 12 wind_station       seq=0   bytes=15 [600C00003F83E278E7113200000001]:   bat=52%  rssi=-60 snr=0  W=9.5/280/14.0  S=275/UV2  F=1
[    1.2s] #3    stn=  8 weather_station    seq=0   bytes=26 [00080000BF766326444C534EE84000EE4647002C004000000008]:   bat=39%  rssi=-72 snr=0  T=10.00 P=987 H=69  W=13.0/167/16.5  R=0/0  S=238/UV4  AQI=142  rad=22/0.08  C=6  F=1
[    1.3s] #4    stn= 16 radiation_monitor  seq=0   bytes=17 [801000003F8BB014400D73CA2E00000001]:   bat=55%  rssi=-64 snr=10  T=17.75 P=998 H=46  rad=81/0.13  F=1
[    1.9s] #5    stn=  1 weather_station    seq=0   bytes=26 [00010000BF7669A74CA2510763000329A60A802C004800000008]:   bat=42%  rssi=-96 snr=0  T=18.25 P=998 H=37  W=4.0/83/6.0  R=0/0  S=809/UV10  AQI=21  rad=22/0.09  C=6  F=1
[    2.2s] #6    stn= 11 water_level        seq=0   bytes=13 [300B00003F9BE552C400000003]:   bat=61%  rssi=-60 snr=0  T=2.50  D=354  F=1
[    2.5s] #7    stn=  4 environment        seq=0   bytes=14 [500400003E91D74DAA0000000010]:   bat=58%  rssi=-92 snr=-10  T=18.25 P=1031 H=32  F=1
[    2.9s] #8    stn= 13 snow_depth         seq=0   bytes=18 [400D0000BE60E9A258B6C0EE52700000001A]:   bat=94%  rssi=-96 snr=0  T=-17.25 P=946 H=59  S=594/UV7  D=150  F=1
[    3.8s] #9    stn= 14 soil_moisture      seq=0   bytes=11 [200E00003FC2E649D0BC04]:   bat=77%  rssi=-76 snr=0  T=10.25  D=47  H=29%  F=1
[    4.0s] #10   stn=  5 wind_station       seq=0   bytes=15 [600500003FEA13F668C3D000000001]:   bat=94%  rssi=-88 snr=-10  W=15.5/252/17.5  S=61/UV0  F=1
[    5.8s] #11   stn=  9 soil_moisture      seq=0   bytes=11 [200900003F9B26FB70B404]:   bat=61%  rssi=-72 snr=0  T=15.75  D=45  H=55%  F=1
[    6.2s] #12   stn=  2 air_quality        seq=0   bytes=23 [100200003F6AD85C93A3378406850401A30C8000000080]:   bat=42%  rssi=-76 snr=-10  T=26.75 P=996 H=58  AQ=102 PM[40/65/50/40] VOC=140 NOx=50  F=1
[    6.4s] #13   stn=  7 air_quality        seq=0   bytes=24 [100700003FB22864A303178184020203AF8E02C000000020]:   bat=71%  rssi=-88 snr=0  T=27.00 P=998 H=48  AQ=98 PM[15/40/20/20] VOC=190 NOx=56 CO2=1100  F=1
[    7.5s] #14   stn= 15 rain_gauge         seq=0   bytes=14 [700F00003FBAE1A16C8000000082]:   bat=74%  rssi=-76 snr=0  T=14.25  R=26/4  F=1
[    7.8s] #15   stn=  6 soil_moisture      seq=0   bytes=11 [200600003FBAE6ACC0D404]:   bat=74%  rssi=-76 snr=0  T=13.25  D=53  H=76%  F=1
[    8.9s] #16   stn= 10 water_level        seq=0   bytes=13 [300A00003FE1A5B1C200000002]:   bat=90%  rssi=-96 snr=0  T=5.50  D=225  F=1
[   13.2s] #17   stn= 12 wind_station       seq=1   bytes=11 [600C00013C816257261162]:   bat=52%  rssi=-100 snr=0  W=9.0/260/12.0  S=278/UV2
[   13.8s] #18   stn=  4 environment        seq=1   bytes=10 [500400013891A74DB1E1]:   bat=58%  rssi=-96 snr=0  T=18.25 P=1032 H=30
[   13.9s] #19   stn=  9 soil_moisture      seq=1   bytes=10 [200900013E9A66FB50B5]:   bat=61%  rssi=-84 snr=0  T=15.75  D=45  H=53%
[   14.1s] #20   stn=  8 weather_station    seq=1   bytes=16 [000800013F6376444C836F870000F843]:   bat=39%  rssi=-68 snr=10  T=10.00 P=987 H=72  W=13.5/174/14.0  R=0/0  S=248/UV4
[   14.3s] #21   stn= 16 radiation_monitor  seq=1   bytes=13 [801000013C8BE013400E73CAAE]:   bat=55%  rssi=-60 snr=0  T=17.75 P=999 H=46  rad=77/0.14
[   14.3s] #22   stn=  7 air_quality        seq=1   bytes=20 [100700013CB258649B23378204018183B10C02C0]:   bat=71%  rssi=-84 snr=-10  T=27.00 P=997 H=50  AQ=102 PM[20/40/15/15] VOC=196 NOx=48 CO2=1100
[   14.7s] #23   stn= 13 snow_depth         seq=1   bytes=13 [400D00013EE9E258B6BEF24084]:   bat=94%  rssi=-92 snr=0  T=-17.25 P=945 H=60  S=576/UV8  D=150
[   14.9s] #24   stn= 14 soil_moisture      seq=1   bytes=10 [200E00013EC26649F0BE]:   bat=77%  rssi=-84 snr=0  T=10.25  D=47  H=31%
[   15.4s] #25   stn=  3 soil_moisture      seq=1   bytes=10 [200300013EC1E75A908A]:   bat=77%  rssi=-92 snr=0  T=18.75  D=34  H=41%
[   15.4s] #26   stn= 11 water_level        seq=1   bytes=9  [300B00013C995552C6]:   bat=61%  rssi=-100 snr=-10  T=2.50  D=355
[   15.6s] #27   stn=  5 wind_station       seq=1   bytes=11 [600500013CEA33F64A0330]:   bat=94%  rssi=-88 snr=10  W=15.5/250/20.0  S=51/UV0
[   15.6s] #28   stn=  6 soil_moisture      seq=1   bytes=10 [200600013EBAA6B4B0D7]:   bat=74%  rssi=-80 snr=0  T=13.50  D=53  H=75%
[   16.1s] #29   stn=  1 weather_station    seq=1   bytes=16 [000100013F6AE74C922107A400031C93]:   bat=42%  rssi=-76 snr=0  T=18.25 P=996 H=34  W=4.0/86/8.0  R=0/0  S=796/UV9
[   18.4s] #30   stn= 15 rain_gauge         seq=1   bytes=10 [700F00013CBA31A16CA1]:   bat=74%  rssi=-88 snr=10  T=14.25  R=26/4
[   18.6s] #31   stn=  2 air_quality        seq=1   bytes=19 [100200013C69F86493C3278406850401A38B02]:   bat=42%  rssi=-92 snr=10  T=27.00 P=996 H=60  AQ=100 PM[40/65/50/40] VOC=142 NOx=44
[   21.0s] #32   stn=  9 soil_moisture      seq=2   bytes=10 [200900023E9A96FB30B7]:   bat=61%  rssi=-80 snr=-10  T=15.75  D=45  H=51%
[   21.5s] #33   stn=  4 environment        seq=2   bytes=10 [500400023891674DB207]:   bat=58%  rssi=-100 snr=0  T=18.25 P=1032 H=32
[   21.6s] #34   stn=  3 soil_moisture      seq=2   bytes=10 [200300023EC1A75AB08B]:   bat=77%  rssi=-96 snr=0  T=18.75  D=34  H=43%
[   22.5s] #35   stn=  6 soil_moisture      seq=2   bytes=10 [200600023EB976B4C0D7]:   bat=74%  rssi=-100 snr=10  T=13.50  D=53  H=76%
[   22.5s] #36   stn= 10 water_level        seq=1   bytes=9  [300A00013CE355B1BC]:   bat=90%  rssi=-68 snr=-10  T=5.50  D=222
[   23.2s] #37   stn=  7 air_quality        seq=2   bytes=20 [100700023CB228649B432F8184018183B28B02C0]:   bat=71%  rssi=-88 snr=0  T=27.00 P=997 H=52  AQ=101 PM[15/40/15/15] VOC=202 NOx=44 CO2=1100
[   24.3s] #38   stn= 11 water_level        seq=2   bytes=9  [300B00023C997552C7]:   bat=61%  rssi=-100 snr=10  T=2.50  D=355
[   25.4s] #39   stn= 16 radiation_monitor  seq=2   bytes=13 [801000023C8A1013400C734B2F]:   bat=55%  rssi=-88 snr=-10  T=17.50 P=1000 H=47  rad=77/0.12
[   25.5s] #40   stn=  5 wind_station       seq=2   bytes=11 [600500023CEB64152AC3B0]:   bat=94%  rssi=-68 snr=0  W=16.0/238/21.5  S=59/UV0
[   25.7s] #41   stn= 14 soil_moisture      seq=2   bytes=10 [200E00023EC1E649D0BF]:   bat=77%  rssi=-92 snr=0  T=10.25  D=47  H=29%
[   27.0s] #42   stn=  9 soil_moisture      seq=3   bytes=10 [200900033E9A270330B7]:   bat=61%  rssi=-88 snr=0  T=16.00  D=45  H=51%
[   27.0s] #43   stn= 13 snow_depth         seq=2   bytes=13 [400D00023EEBA258B8BEEA507F]:   bat=94%  rssi=-64 snr=0  T=-17.00 P=945 H=58  S=592/UV7  D=150
[   27.2s] #44   stn= 12 wind_station       seq=2   bytes=11 [600C00023C82A256C790E3]:   bat=52%  rssi=-80 snr=0  W=9.0/256/15.0  S=270/UV3
[   28.7s] #45   stn=  8 weather_station    seq=2   bytes=16 [000800023F61763C4CB35027C000F653]:   bat=39%  rssi=-100 snr=10  T=9.75 P=987 H=75  W=13.0/181/15.5  R=0/0  S=246/UV5
[   30.4s] #46   stn=  1 weather_station    seq=2   bytes=16 [000100023F6AF744A211488380031893]:   bat=42%  rssi=-76 snr=10  T=18.00 P=998 H=33  W=5.0/96/7.0  R=0/0  S=792/UV9
[   30.7s] #47   stn=  7 air_quality        seq=3   bytes=20 [100700033CB3286493333F8184818183B18902C0]:   bat=71%  rssi=-72 snr=0  T=27.00 P=996 H=51  AQ=103 PM[15/45/15/15] VOC=198 NOx=36 CO2=1100
[   31.8s] #48   stn= 10 water_level        seq=2   bytes=9  [300A00023CE265B1C1]:   bat=90%  rssi=-84 snr=0  T=5.50  D=224
[   31.8s] #49   stn=  6 soil_moisture      seq=3   bytes=10 [200600033EB966B4C0D7]:   bat=74%  rssi=-100 snr=0  T=13.50  D=53  H=76%
[   32.2s] #50   stn=  2 air_quality        seq=2   bytes=19 [100200023C6A286493C31F8486050481A38C02]:   bat=42%  rssi=-88 snr=0  T=27.00 P=996 H=60  AQ=99 PM[45/60/50/45] VOC=142 NOx=48
[   32.4s] #51   stn= 14 soil_moisture      seq=3   bytes=10 [200E00033EC3B651B0BF]:   bat=77%  rssi=-64 snr=10  T=10.50  D=47  H=27%
[   32.7s] #52   stn= 15 rain_gauge         seq=2   bytes=10 [700F00023CBB21A26CBF]:   bat=74%  rssi=-72 snr=0  T=14.25  R=26/8
[   32.7s] #53   stn= 11 water_level        seq=3   bytes=9  [300B00033C916552CA]:   bat=58%  rssi=-100 snr=0  T=2.50  D=357
[   33.4s] #54   stn=  3 soil_moisture      seq=3   bytes=10 [200300033EC2E75AB08B]:   bat=77%  rssi=-76 snr=0  T=18.75  D=34  H=43%
[   33.5s] #55   stn= 16 radiation_monitor  seq=3   bytes=13 [801000033C8B6012800A734BB0]:   bat=55%  rssi=-68 snr=0  T=17.50 P=1001 H=48  rad=74/0.10
[   34.5s] #56   stn=  4 environment        seq=3   bytes=10 [500400033893A74DBA2A]:   bat=58%  rssi=-64 snr=0  T=18.25 P=1033 H=34
[   34.6s] #57   stn= 13 snow_depth         seq=3   bytes=13 [400D00033EEA7258B8BCE25280]:   bat=94%  rssi=-84 snr=10  T=-17.00 P=944 H=56  S=594/UV8  D=150
[   36.9s] #58   stn=  9 soil_moisture      seq=4   bytes=10 [200900043E9B570310B4]:   bat=61%  rssi=-68 snr=-10  T=16.00  D=45  H=49%
[   39.0s] #59   stn= 12 wind_station       seq=3   bytes=11 [600C00033C831295065142]:   bat=52%  rssi=-72 snr=-10  W=10.0/236/12.5  S=276/UV2
[   39.6s] #60   stn=  4 environment        seq=4   bytes=10 [500400043891674DBA41]:   bat=58%  rssi=-100 snr=0  T=18.25 P=1033 H=36
[   40.0s] #61   stn= 16 radiation_monitor  seq=4   bytes=13 [801000043C8AD013800B734C2E]:   bat=55%  rssi=-76 snr=-10  T=17.50 P=1002 H=46  rad=78/0.11
[   40.3s] #62   stn=  5 wind_station       seq=3   bytes=11 [600500033CEBE415890270]:   bat=94%  rssi=-60 snr=0  W=16.0/242/18.0  S=39/UV0
[   40.3s] #63   stn=  6 soil_moisture      seq=4   bytes=10 [200600043EBB16ACB0D6]:   bat=74%  rssi=-72 snr=-10  T=13.25  D=53  H=75%
[   41.1s] #64   stn=  2 air_quality        seq=3   bytes=19 [100200033C69A85C9BA2E78485858481A28C02]:   bat=42%  rssi=-96 snr=0  T=26.75 P=997 H=58  AQ=92 PM[45/55/55/45] VOC=138 NOx=48
[   41.1s] #65   stn=  3 soil_moisture      seq=4   bytes=10 [200300043EC2A75AD08A]:   bat=77%  rssi=-80 snr=0  T=18.75  D=34  H=45%
[   41.6s] #66   stn= 11 water_level        seq=4   bytes=9  [300B00043C92F552C6]:   bat=58%  rssi=-76 snr=10  T=2.50  D=355
[   41.8s] #67   stn= 15 rain_gauge         seq=3   bytes=10 [700F00033CB9A1A26C0A]:   bat=74%  rssi=-96 snr=0  T=14.00  R=26/8
[   42.3s] #68   stn= 10 water_level        seq=3   bytes=9  [300A00033CE235B1BC]:   bat=90%  rssi=-88 snr=10  T=5.50  D=222
[   42.3s] #69   stn= 13 snow_depth         seq=4   bytes=13 [400D00043EEB5260B8BADA4495]:   bat=94%  rssi=-68 snr=-10  T=-17.00 P=943 H=54  S=580/UV9  D=152
[   42.4s] #70   stn=  1 weather_station    seq=3   bytes=16 [000100033F6A2744B21148E300031281]:   bat=42%  rssi=-88 snr=0  T=18.00 P=1000 H=33  W=5.0/100/6.0  R=0/0  S=786/UV8
[   43.4s] #71   stn=  8 weather_station    seq=3   bytes=16 [000800033F62663C4CD3512700010941]:   bat=39%  rssi=-84 snr=0  T=9.75 P=987 H=77  W=13.0/193/14.0  R=0/0  S=265/UV4
[   45.6s] #72   stn=  7 air_quality        seq=4   bytes=20 [100700043CB2E8649B532F8204018103B10782E0]:   bat=71%  rssi=-76 snr=0  T=27.00 P=997 H=53  AQ=101 PM[20/40/15/10] VOC=196 NOx=30 CO2=1150
[   46.4s] #73   stn= 14 soil_moisture      seq=4   bytes=10 [200E00043EC22651C0BF]:   bat=77%  rssi=-88 snr=0  T=10.50  D=47  H=28%
[   46.7s] #74   stn=  2 air_quality        seq=4   bytes=19 [100200043C6AE8649B92CF8406050501A08D82]:   bat=42%  rssi=-76 snr=0  T=27.00 P=997 H=57  AQ=89 PM[40/60/50/50] VOC=130 NOx=54
[   48.2s] #75   stn=  6 soil_moisture      seq=5   bytes=10 [200600053EBBA6B4D0D6]:   bat=74%  rssi=-64 snr=0  T=13.50  D=53  H=77%
[   48.7s] #76   stn=  9 soil_moisture      seq=5   bytes=10 [200900053E9B970310B6]:   bat=61%  rssi=-64 snr=-10  T=16.00  D=45  H=49%
[   49.0s] #77   stn= 10 water_level        seq=4   bytes=9  [300A00043CE325B1C0]:   bat=90%  rssi=-72 snr=0  T=5.50  D=224
[   49.4s] #78   stn=  5 wind_station       seq=4   bytes=11 [600500043CE9D434C9C390]:   bat=94%  rssi=-92 snr=-10  W=16.5/233/19.5  S=57/UV0
[   50.3s] #79   stn=  4 environment        seq=5   bytes=10 [500400053891A74DC243]:   bat=58%  rssi=-96 snr=0  T=18.25 P=1034 H=36
[   51.2s] #80   stn=  7 air_quality        seq=5   bytes=20 [100700053CB17864A3334F8184010083B18902E0]:   bat=71%  rssi=-100 snr=10  T=27.00 P=998 H=51  AQ=105 PM[15/40/10/5] VOC=198 NOx=36 CO2=1150
[   53.4s] #81   stn= 13 snow_depth         seq=5   bytes=13 [400D00053EEB5260B8BAD65894]:   bat=94%  rssi=-68 snr=-10  T=-17.00 P=943 H=53  S=600/UV9  D=152
[   53.5s] #82   stn= 12 wind_station       seq=4   bytes=11 [600C00043C8152B5665062]:   bat=52%  rssi=-100 snr=-10  W=10.5/240/12.5  S=262/UV2
[   54.5s] #83   stn= 16 radiation_monitor  seq=5   bytes=13 [801000053C8A2013000D734CB0]:   bat=55%  rssi=-88 snr=0  T=17.50 P=1003 H=48  rad=76/0.13
[   54.8s] #84   stn= 15 rain_gauge         seq=4   bytes=10 [700F00043CBA21A26C8D]:   bat=74%  rssi=-88 snr=0  T=14.25  R=26/8
[   55.0s] #85   stn=  2 air_quality        seq=5   bytes=19 [100200053C6BA864A382CF8485850501A08E82]:   bat=42%  rssi=-64 snr=0  T=27.00 P=998 H=56  AQ=89 PM[45/55/50/50] VOC=130 NOx=58
[   55.2s] #86   stn= 10 water_level        seq=5   bytes=9  [300A00053CE1D5B1C1]:   bat=90%  rssi=-92 snr=-10  T=5.50  D=224
[   55.3s] #87   stn=  5 wind_station       seq=5   bytes=11 [600500053CE96453AA0371]:   bat=94%  rssi=-100 snr=0  W=17.0/221/20.0  S=55/UV1
[   55.6s] #88   stn= 11 water_level        seq=5   bytes=9  [300B00053C91B552C2]:   bat=58%  rssi=-96 snr=10  T=2.50  D=353
[   55.9s] #89   stn=  3 soil_moisture      seq=5   bytes=10 [200300053EC2A75AD08B]:   bat=77%  rssi=-80 snr=0  T=18.75  D=34  H=45%
[   56.4s] #90   stn=  1 weather_station    seq=4   bytes=16 [000100043F69673CAA4149E480032A91]:   bat=42%  rssi=-100 snr=0  T=17.75 P=999 H=36  W=5.0/111/9.0  R=0/0  S=810/UV9
[   57.5s] #91   stn=  4 environment        seq=6   bytes=10 [50040006388B374DBA61]:   bat=55%  rssi=-72 snr=10  T=18.25 P=1033 H=38
[   57.5s] #92   stn=  6 soil_moisture      seq=6   bytes=10 [200600063EBAA6B4C0D5]:   bat=74%  rssi=-80 snr=0  T=13.50  D=53  H=76%
[   57.9s] #93   stn=  7 air_quality        seq=6   bytes=20 [100700063CB1D85C9B134F8184010083B28782E0]:   bat=71%  rssi=-92 snr=-10  T=26.75 P=997 H=49  AQ=105 PM[15/40/10/5] VOC=202 NOx=30 CO2=1150
[   58.3s] #94   stn=  8 weather_station    seq=4   bytes=16 [000800043F6266344CE3508740010843]:   bat=39%  rssi=-84 snr=0  T=9.50 P=987 H=78  W=13.0/186/14.5  R=0/0  S=264/UV4
[   59.6s] #95   stn= 16 radiation_monitor  seq=6   bytes=13 [801000063C8A2012400B734CB2]:   bat=55%  rssi=-88 snr=0  T=17.50 P=1003 H=50  rad=73/0.11
[   59.8s] #96   stn= 14 soil_moisture      seq=5   bytes=10 [200E00053EC25651E0BF]:   bat=77%  rssi=-84 snr=-10  T=10.50  D=47  H=30%
[   60.2s] #97   stn=  2 air_quality        seq=6   bytes=19 [100200063C69A864AB72B784860505019E9002]:   bat=42%  rssi=-96 snr=0  T=27.00 P=999 H=55  AQ=86 PM[45/60/50/50] VOC=122 NOx=64
[   61.5s] #98   stn=  9 soil_moisture      seq=6   bytes=10 [200900063E9A570300B6]:   bat=61%  rssi=-84 snr=-10  T=16.00  D=45  H=48%
[   61.9s] #99   stn= 15 rain_gauge         seq=5   bytes=10 [700F00053CBAE1A26CB6]:   bat=74%  rssi=-76 snr=0  T=14.25  R=26/8
[   62.8s] #100  stn=  1 weather_station    seq=5   bytes=16 [000100053F6AD744B2216A0440033F81]:   bat=42%  rssi=-76 snr=-10  T=18.00 P=1000 H=34  W=5.5/113/8.5  R=0/0  S=831/UV8

=== 100 packets in 62.9s simulated ===

  ID  Variant             TXs  Bat%  Last seq
  --  ------------------  ---  ----  --------
   0  weather_station       6   43%  6
   1  air_quality           7   42%  7
   2  soil_moisture         6   76%  6
   3  environment           7   56%  7
   4  wind_station          6   93%  6
   5  soil_moisture         7   73%  7
   6  air_quality           7   72%  7
   7  weather_station       5   40%  5
   8  soil_moisture         7   60%  7
   9  water_level           6   90%  6
  10  water_level           6   59%  6
  11  wind_station          5   52%  5
  12  snow_depth            6   94%  6
  13  soil_moisture         6   78%  6
  14  rain_gauge            6   75%  6
  15  radiation_monitor     7   56%  7
```

## simulator_sensor_lora_esp32/ — ESP32 LoRa Transmitter

The same multi-sensor simulator running on an ESP32-C3, transmitting iotdata
packets over LoRa via an EBYTE E22-xxxTxxD module (DIP variant). No real sensors
— all readings are generated by the simulator, but packets go out over the air
exactly as a real sensor deployment would.

The firmware is built with ESP-IDF and uses an encode-only, integer-only
(`IOTDATA_NO_FLOATING`) configuration to minimise code size. The simulator's
internal centi-unit representation maps directly to the integer-mode encoder.

Requires the E22 radio driver installed at `/opt/e22900t22u`:
[github.com/matthewgream/e22900t22u](https://github.com/matthewgream/e22900t22u).

Build and flash:

```bash
cd simulator_sensor_lora_esp32
. ${IDF_PATH}/export.sh
idf.py set-target esp32c3
idf.py build
idf.py flash -p /dev/ttyACM0 monitor
```

```text
# idf.py flash -p /dev/ttyACM0 monitor
Executing action: flash
Running ninja in directory /opt/libiotdata/examples/simulator_sensor_lora_esp32/build
...
Serial port /dev/ttyACM0
Connecting...
Chip is ESP32-C3 (QFN32) (revision v0.4)
Features: WiFi, BLE, Embedded Flash 4MB (XMC)
Crystal is 40MHz
...
Hard resetting via RTS pin...
Executing action: monitor
Running idf_monitor in directory /opt/libiotdata/examples/simulator_sensor_lora_esp32
...
ESP-ROM:esp32c3-api1-20210207
Build:Feb  7 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0xd (SPI_FAST_FLASH_BOOT)
Saved PC:0x40381db4
--- 0x40381db4: rv_utils_wait_for_intr at /opt/esp32/esp-idf/components/riscv/include/riscv/rv_utils.h:79
--- (inlined by) esp_cpu_wait_for_intr at /opt/esp32/esp-idf/components/esp_hw_support/cpu.c:62
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fcd5830,len:0x15ec
load:0x403cbf10,len:0xc44
--- 0x403cbf10: esp_bootloader_get_description at /opt/esp32/esp-idf/components/esp_bootloader_format/esp_bootloader_desc.c:39
load:0x403ce710,len:0x3020
--- 0x403ce710: esp_flash_encryption_enabled at /opt/esp32/esp-idf/components/bootloader_support/src/flash_encrypt.c:89
entry 0x403cbf1a
--- 0x403cbf1a: call_start_cpu0 at /opt/esp32/esp-idf/components/bootloader/subproject/main/bootloader_start.c:25
I (15) boot: ESP-IDF v5.5.2 2nd stage bootloader
I (15) boot: compile time Feb 27 2026 16:08:51
...
I (55) cpu_start: Unicore app
I (63) cpu_start: Pro cpu start user code
I (63) cpu_start: cpu freq: 160000000 Hz
I (64) app_init: Application information:
I (64) app_init: Project name:     simulator_sensor_lora_esp32
I (64) app_init: App version:      4abddb0
I (64) app_init: Compile time:     Feb 27 2026 16:08:42
I (64) app_init: ELF file SHA256:  44a38928c...
I (64) app_init: ESP-IDF:          v5.5.2
I (64) efuse_init: Min chip rev:     v0.3
I (64) efuse_init: Max chip rev:     v1.99
I (65) efuse_init: Chip rev:         v0.4
...
I (68) main_task: Calling app_main()
I (3068) app: iotdata multi-sensor simulator transmitter
I (3068) app: device: e22 connected
device: product_info: name=0022, version=16, maxpower=22, frequency=11, type=0 [00 22 10 16 0B 00 00]
device: module_config: address=0x0008, network=0x00, channel=23 (frequency=873.125MHz), data-rate=2.4kbps (Default), packet-size=240bytes (Default), transmit-power=22dBm (Default), encryption-key=0x0000, rssi-channel=on, rssi-packet=on, mode-listen-before-tx=on, mode-transmit=fixed-point, mode-relay=off, mode-wor-enable=off, mode-wor-cycle=2000ms (Default), uart-rate=9600bps (Default), uart-parity=8N1 (Default)
I (3328) app: device: e22 configured, transfer mode active
I (3328) app: simulator: 16 sensors, seed=0xCE0D104D
I (3328) app:   [ 0] rain_gauge         stn=1    bat=69%
I (3328) app:   [ 1] soil_moisture      stn=2    bat=44%
I (3328) app:   [ 2] radiation_monitor  stn=3    bat=91%
I (3328) app:   [ 3] water_level        stn=4    bat=74%
I (3328) app:   [ 4] soil_moisture      stn=5    bat=86%
I (3328) app:   [ 5] environment        stn=6    bat=66%
I (3328) app:   [ 6] weather_station    stn=7    bat=47%
I (3328) app:   [ 7] air_quality        stn=8    bat=43%
I (3328) app:   [ 8] air_quality        stn=9    bat=50%
I (3328) app:   [ 9] rain_gauge         stn=10   bat=90%
I (3328) app:   [10] radiation_monitor  stn=11   bat=84%
I (3328) app:   [11] soil_moisture      stn=12   bat=87%
I (3328) app:   [12] snow_depth         stn=13   bat=84%
I (3328) app:   [13] wind_station       stn=14   bat=52%
I (3328) app:   [14] weather_station    stn=15   bat=70%
I (3328) app:   [15] air_quality        stn=16   bat=59%
device: e22 tx #000000: stn=11   radiation_monitor  seq=000000 bytes=17  hex: 80 0B 00 00 2F D0 03 C0 05 9C D6 0B 80 00 00 00 50
device: e22 tx #000001: stn=1    rain_gauge         seq=000000 bytes=13  hex: 70 01 00 00 2F A8 38 1E 00 00 00 00 20
device: e22 tx #000002: stn=10   rain_gauge         seq=000000 bytes=13  hex: 70 0A 00 00 2F E0 68 54 80 00 00 00 20
device: e22 tx #000003: stn=12   soil_moisture      seq=000000 bytes=10  hex: 20 0C 00 00 2F D9 A6 AC 2F 01
device: e22 tx #000004: stn=15   weather_station    seq=000000 bytes=31  hex: 00 0F 00 00 AF 7E B1 AB 5E D4 60 10 F1 11 72 10 91 60 19 80 0B 01 B0 0F 00 41 C4 00 00 00 02
device: channel_rssi_read: failed, received 0 bytes, expected 4 bytes
device: e22 tx #000005: stn=2    soil_moisture      seq=000000 bytes=11  hex: 20 02 00 00 3F 71 F6 C4 C0 84 04
device: e22 tx #000006: stn=6    environment        seq=000000 bytes=13  hex: 50 06 00 00 2E A1 FD 74 E4 00 00 00 05
device: e22 tx #000007: stn=14   wind_station       seq=000000 bytes=15  hex: 60 0E 00 00 2F 80 31 41 12 D6 C0 00 00 00 72
device: e22 tx #000008: stn=9    air_quality        seq=000000 bytes=22  hex: 10 09 00 00 2F 79 D9 22 88 89 E0 20 21 01 20 6B A6 00 00 00 00 21
device: e22 tx #000009: stn=5    soil_moisture      seq=000000 bytes=10  hex: 20 05 00 00 2F D9 A2 60 3A 01
device: e22 tx #000010: stn=8    air_quality        seq=000000 bytes=22  hex: 10 08 00 00 2F 6A 19 3E C4 B3 E1 01 61 60 E0 6D E5 00 00 00 00 21
device: e22 tx #000011: stn=10   rain_gauge         seq=000001 bytes=9   hex: 70 0A 00 01 2C E0 68 54 A4
device: e22 tx #000012: stn=13   snow_depth         seq=000000 bytes=23  hex: 40 0D 00 00 AF 60 D0 1D 43 86 BB 98 DA 03 7B 4A 00 6D 38 00 00 00 04
device: e22 tx #000013: stn=3    radiation_monitor  seq=000000 bytes=17  hex: 80 03 00 00 2F E0 03 80 07 DA B4 09 80 00 00 00 40
device: e22 tx #000014: stn=6    environment        seq=000001 bytes=9   hex: 50 06 00 01 28 A1 FD 76 EB
device: e22 tx #000015: stn=16   air_quality        seq=000000 bytes=23  hex: 10 10 00 00 3F 93 87 64 C9 F1 47 85 02 08 87 81 AD 1A 00 00 00 00 84
device: e22 tx #000016: stn=1    rain_gauge         seq=000001 bytes=9   hex: 70 01 00 01 2C A8 38 1D E9
device: e22 tx #000017: stn=12   soil_moisture      seq=000001 bytes=9   hex: 20 0C 00 01 2E D9 A6 B4 2F
device: e22 tx #000018: stn=4    water_level        seq=000000 bytes=13  hex: 30 04 00 00 2F B9 96 6F 00 00 00 00 82
device: e22 tx #000019: stn=7    weather_station    seq=000000 bytes=31  hex: 00 07 00 00 AF 7E 79 B9 46 70 0B F0 60 00 00 04 CC C0 16 00 29 01 B0 0F 00 41 C4 00 00 00 02
device: e22 tx #000020: stn=14   wind_station       seq=000001 bytes=11  hex: 60 0E 00 01 2C 80 39 08 F2 A6 8B
device: e22 tx #000021: stn=2    soil_moisture      seq=000001 bytes=9   hex: 20 02 00 01 2E 71 B1 34 21
device: e22 tx #000022: stn=1    rain_gauge         seq=000002 bytes=9   hex: 70 01 00 02 2C A8 38 1E 01
device: e22 tx #000023: stn=15   weather_station    seq=000001 bytes=15  hex: 00 0F 00 01 2F B1 AD 60 D4 57 D1 01 11 79 0C
device: e22 tx #000024: stn=11   radiation_monitor  seq=000001 bytes=13  hex: 80 0B 00 01 2C D0 03 F0 05 9C D6 2B 91
device: e22 tx #000025: stn=6    environment        seq=000002 bytes=9   hex: 50 06 00 02 28 A1 FF 76 E1
device: e22 tx #000026: stn=3    radiation_monitor  seq=000001 bytes=13  hex: 80 03 00 01 2C E0 03 90 07 5A B4 2A 11
device: e22 tx #000027: stn=5    soil_moisture      seq=000001 bytes=10  hex: 20 05 00 01 3E DA 86 89 90 EA
device: e22 tx #000028: stn=16   air_quality        seq=000001 bytes=18  hex: 10 10 00 01 2C 91 D9 32 84 45 E1 20 82 41 C0 6B 26 20
device: e22 tx #000029: stn=10   rain_gauge         seq=000002 bytes=9   hex: 70 0A 00 02 2C E0 4C 54 A4
device: e22 tx #000030: stn=8    air_quality        seq=000001 bytes=18  hex: 10 08 00 01 2C 6A 1B 40 BC B5 E1 21 41 81 00 6D C4 80
device: e22 tx #000031: stn=9    air_quality        seq=000001 bytes=18  hex: 10 09 00 01 2C 79 D7 22 88 91 E0 00 20 E1 00 6B 46 00
device: e22 tx #000032: stn=7    weather_station    seq=000001 bytes=15  hex: 00 07 00 01 2F 79 BB 44 64 04 08 20 00 00 80
device: e22 tx #000033: stn=2    soil_moisture      seq=000002 bytes=9   hex: 20 02 00 02 2E 71 B1 34 21
device: e22 tx #000034: stn=12   soil_moisture      seq=000002 bytes=9   hex: 20 0C 00 02 2E D9 A6 AC 2F
device: e22 tx #000035: stn=13   snow_depth         seq=000001 bytes=12  hex: 40 0D 00 01 2E D0 1D 43 06 3B 95 DC
device: e22 tx #000036: stn=4    water_level        seq=000001 bytes=9   hex: 30 04 00 01 2C B9 96 6E 86
device: e22 tx #000037: stn=11   radiation_monitor  seq=000002 bytes=13  hex: 80 0B 00 02 2C D0 03 B0 05 DC D6 2B 80
device: e22 tx #000038: stn=3    radiation_monitor  seq=000002 bytes=13  hex: 80 03 00 02 2C E0 03 90 06 DA B4 0A 40
device: e22 tx #000039: stn=6    environment        seq=000003 bytes=9   hex: 50 06 00 03 28 A1 FF 78 E6
device: e22 tx #000040: stn=14   wind_station       seq=000002 bytes=11  hex: 60 0E 00 02 2C 80 48 A8 F2 42 B4
device: e22 tx #000041: stn=9    air_quality        seq=000002 bytes=18  hex: 10 09 00 02 2C 79 D5 20 88 9B E0 00 20 C1 20 6A E6 60
device: e22 tx #000042: stn=1    rain_gauge         seq=000003 bytes=10  hex: 70 01 00 03 3C AA A0 E0 78 9B
device: e22 tx #000043: stn=5    soil_moisture      seq=000002 bytes=10  hex: 20 05 00 02 3E DA 56 89 90 EB
device: e22 tx #000044: stn=15   weather_station    seq=000002 bytes=16  hex: 00 0F 00 02 3F B3 D6 AD 7B 81 7E C4 C4 45 D9 3A
device: e22 tx #000045: stn=11   radiation_monitor  seq=000003 bytes=13  hex: 80 0B 00 03 2C D0 03 60 06 5C D6 0B 04
device: e22 tx #000046: stn=12   soil_moisture      seq=000003 bytes=9   hex: 20 0C 00 03 2E D9 A6 B0 2F
device: e22 tx #000047: stn=6    environment        seq=000004 bytes=9   hex: 50 06 00 04 28 A1 FF 7A EB
device: e22 tx #000048: stn=10   rain_gauge         seq=000003 bytes=9   hex: 70 0A 00 03 2C E0 4C 54 8B
device: e22 tx #000049: stn=4    water_level        seq=000002 bytes=9   hex: 30 04 00 02 2C B9 96 6F 0B
I (34058) app: status: tx=50 errors=0 rssi=-69 dBm uptime=30s
device: e22 tx #000050: stn=7    weather_station    seq=000002 bytes=16  hex: 00 07 00 02 3F 7A A6 F5 09 80 11 61 80 00 00 0A
device: e22 tx #000051: stn=16   air_quality        seq=000002 bytes=18  hex: 10 10 00 02 2C 91 DB 30 8C 53 E1 20 82 41 E0 6B 05 E0
device: e22 tx #000052: stn=2    soil_moisture      seq=000003 bytes=9   hex: 20 02 00 03 2E 71 B1 34 21
device: e22 tx #000053: stn=8    air_quality        seq=000002 bytes=18  hex: 10 08 00 02 2C 6A 1B 40 B8 B7 E1 21 41 61 20 6D C4 80
device: e22 tx #000054: stn=13   snow_depth         seq=000002 bytes=12  hex: 40 0D 00 02 2E D0 1C 43 85 BC 92 61
device: e22 tx #000055: stn=10   rain_gauge         seq=000004 bytes=9   hex: 70 0A 00 04 2C E0 4C 54 85
device: e22 tx #000056: stn=5    soil_moisture      seq=000003 bytes=10  hex: 20 05 00 03 3E D9 56 89 A0 E8
device: e22 tx #000057: stn=3    radiation_monitor  seq=000003 bytes=13  hex: 80 03 00 03 2C E0 03 40 06 9A 94 0A 01
device: e22 tx #000058: stn=9    air_quality        seq=000003 bytes=18  hex: 10 09 00 03 2C 79 D5 20 8C 8D E0 00 20 E1 00 6B 06 00
device: e22 tx #000059: stn=14   wind_station       seq=000003 bytes=11  hex: 60 0E 00 03 2C 80 40 59 12 56 E0
device: e22 tx #000060: stn=11   radiation_monitor  seq=000004 bytes=13  hex: 80 0B 00 04 2C D0 03 60 05 DC D5 EA A0
```

## gateway_mqtt_lora_linux/ — Linux LoRa-to-MQTT Gateway

A Linux gateway that receives iotdata packets from an EBYTE E22-900T22U module
(USB variant), decodes them to JSON, and publishes to MQTT. The variant byte in
each packet header determines the MQTT topic automatically:
`<prefix>/<variant_name>/<station_id>` (where station_id is the 4-digit 0-padded
hex encoded 16-bit station identifier).

Features:

- **Mesh support**: when enabled, the gateway participates in the mesh protocol
  — it originates beacons, unwraps forwarded packets, sends ACKs to relaying
  nodes, and logs all mesh control traffic. Direct and mesh-relayed packets are
  both deduplicated via the ring buffer.
- **Cross-gateway UDP dedup**: independently of mesh, multiple gateways with
  overlapping radio coverage can synchronise their dedup state over UDP. Each
  gateway broadcasts recently-seen `{station_id, sequence}` pairs to its
  configured peers, preventing the same packet from being published to MQTT by
  more than one gateway. Configurable batching delay and peer list.
- **Statistics**: periodic logging of packet rates, RSSI/SNR (channel and
  per-packet EMA), mesh counters, dedup counters, and MQTT connection state.

Requires the E22 radio driver installed at `/opt/e22900t22u`:
[github.com/matthewgream/e22900t22u](https://github.com/matthewgream/e22900t22u).
Also requires `libmosquitto` and `libcjson`.

Build and run:

```bash
cd gateway_mqtt_lora_linux
make
./iotdata_gateway --config iotdata_gateway.cfg
```

Configuration is via a config file and/or command-line overrides (command-line
takes precedence). See `iotdata_gateway.cfg` for the available parameters. A
systemd service file is included; `make install` installs and enables it.

```text
# ./iotdata_gateway --debug true
starting (iotdata gateway: variants=9, features=mesh,dedup)
config: file='iotdata_gateway.cfg', address='0x0008', network='0x00', channel='0x17', rssi-packet='true', rssi-channel='true', listen-before-transmit='true', interval-stat='300', interval-rssi='60', mqtt-client='iot_gwy_01', mqtt-server='mqtt://localhost:1883', mqtt-topic-prefix='iotdata', mesh-enable='false', mesh-station-id='1', mesh-beacon-interval='60', dedup-enable='false', dedup-port='9876', dedup-peers='192.168.0.2:9876,192.168.0.3:9876', dedup-delay='20', debug='true'
config: serial: port=/dev/e22900t22u, rate=9600, bits=8N1
config: e22900t22u: address=0x0008, network=0x00, channel=23, packet-size=240, packet-rate=2, rssi-channel=on, rssi-packet=on, mode-listen-before-tx=on, read-timeout-command=1000, read-timeout-packet=5000, crypt=0x0000, transmit-power=0, transmission-method=transparent, mode-relay=off, debug=off
config: mqtt: client=iot_gwy_01, server=mqtt://localhost:1883, tls-insecure=off, synchronous=off, reconnect-delay=5, reconnect-delay-max=60
config: mesh: enabled=n, station=0x0001, beacon-interval=60, debug=off
config: dedup: enabled=n, port=9876, peers=192.168.0.2:9876,192.168.0.3:9876, delay=20ms, debug=off
device: connect success (port=/dev/e22900t22u, rate=9600, bits=8N1)
device: product_info: name=0022, version=32, maxpower=22, frequency=11, type=0 [00 22 20 16 0B 00 00]
device: module_config: address=0x0008, network=0x00, channel=23 (frequency=873.125MHz), data-rate=2.4kbps (Default), packet-size=240bytes (Default), transmit-power=22dBm (Default), encryption-key=0x0000, rssi-channel=on, rssi-packet=on, mode-listen-before-tx=on, mode-transmit=fixed-point, mode-relay=off, uart-rate=9600bps (Default), uart-parity=8N1 (Default), switch-config-serial=on
mqtt: connecting (host='localhost', port=1883, ssl=false, client='iot_gwy_01')
mesh: disabled, not starting
dedup: disabled, not starting
process: iotdata gateway (stat=300s, rssi=60s [packets=y, channel=y], topic-prefix=iotdata)
process: variant[0] = "weather_station" (pres_bytes=2) -> iotdata/weather_station/<station>
process: variant[1] = "air_quality" (pres_bytes=1) -> iotdata/air_quality/<station>
process: variant[2] = "soil_moisture" (pres_bytes=1) -> iotdata/soil_moisture/<station>
process: variant[3] = "water_level" (pres_bytes=1) -> iotdata/water_level/<station>
process: variant[4] = "snow_depth" (pres_bytes=2) -> iotdata/snow_depth/<station>
process: variant[5] = "environment" (pres_bytes=1) -> iotdata/environment/<station>
process: variant[6] = "wind_station" (pres_bytes=1) -> iotdata/wind_station/<station>
process: variant[7] = "rain_gauge" (pres_bytes=1) -> iotdata/rain_gauge/<station>
process: variant[8] = "radiation_monitor" (pres_bytes=1) -> iotdata/radiation_monitor/<station>
mqtt: connected
  -> iotdata/radiation_monitor/11 (252 bytes)
  -> iotdata/rain_gauge/1 (181 bytes)
  -> iotdata/rain_gauge/10 (181 bytes)
  -> iotdata/soil_moisture/12 (167 bytes)
  -> iotdata/weather_station/15 (473 bytes)
  -> iotdata/soil_moisture/2 (192 bytes)
  -> iotdata/environment/6 (202 bytes)
process: decode failed: Decoding buffer too short for content (variant=1, station=0x0009, size=19)
  -> iotdata/weather_station/0 (164 bytes)
  -> iotdata/air_quality/8 (299 bytes)
  -> iotdata/rain_gauge/10 (160 bytes)
  -> iotdata/snow_depth/13 (336 bytes)
  -> iotdata/radiation_monitor/3 (253 bytes)
  -> iotdata/environment/6 (177 bytes)
  -> iotdata/air_quality/16 (331 bytes)
  -> iotdata/rain_gauge/1 (160 bytes)
  -> iotdata/soil_moisture/12 (156 bytes)
  -> iotdata/water_level/4 (167 bytes)
  -> iotdata/weather_station/7 (467 bytes)
  -> iotdata/wind_station/14 (205 bytes)
  -> iotdata/soil_moisture/2 (152 bytes)
  -> iotdata/rain_gauge/1 (157 bytes)
  -> iotdata/weather_station/15 (295 bytes)
  -> iotdata/radiation_monitor/11 (228 bytes)
  -> iotdata/environment/6 (178 bytes)
  -> iotdata/radiation_monitor/3 (229 bytes)
  -> iotdata/soil_moisture/5 (186 bytes)
  -> iotdata/air_quality/16 (277 bytes)
  -> iotdata/rain_gauge/10 (160 bytes)
  -> iotdata/air_quality/8 (279 bytes)
  -> iotdata/air_quality/9 (276 bytes)
  -> iotdata/weather_station/7 (292 bytes)
  -> iotdata/soil_moisture/2 (152 bytes)
  -> iotdata/soil_moisture/12 (156 bytes)
  -> iotdata/snow_depth/13 (237 bytes)
  -> iotdata/water_level/4 (143 bytes)
  -> iotdata/radiation_monitor/11 (229 bytes)
  -> iotdata/radiation_monitor/3 (229 bytes)
  -> iotdata/environment/6 (178 bytes)
  -> iotdata/wind_station/14 (205 bytes)
  -> iotdata/air_quality/9 (276 bytes)
  -> iotdata/rain_gauge/1 (189 bytes)
  -> iotdata/soil_moisture/5 (186 bytes)
  -> iotdata/weather_station/15 (330 bytes)
  -> iotdata/radiation_monitor/11 (214 bytes)
  -> iotdata/soil_moisture/12 (156 bytes)
  -> iotdata/environment/6 (178 bytes)
  -> iotdata/rain_gauge/10 (157 bytes)
  -> iotdata/water_level/4 (143 bytes)
  -> iotdata/weather_station/7 (319 bytes)
  -> iotdata/air_quality/16 (280 bytes)
  -> iotdata/soil_moisture/2 (152 bytes)
process: decode failed: Decoding buffer too short for content (variant=1, station=0x0008, size=15)
process: packet too short for iotdata header (size=3)
  -> iotdata/snow_depth/13 (238 bytes)
  -> iotdata/rain_gauge/10 (157 bytes)
  -> iotdata/soil_moisture/5 (187 bytes)
  -> iotdata/radiation_monitor/3 (226 bytes)
  -> iotdata/air_quality/9 (275 bytes)
  -> iotdata/radiation_monitor/11 (229 bytes)
```

```text
# mosquitto_sub -t iotdata/#
{"variant":8,"station":11,"sequence":0,"packed_bits":130,"packed_bytes":17,"battery":{"level":84,"charging":false},"radiation":{"cpm":60,"dose":0.2199999988079071},"environment":{"temperature":17.5,"pressure":1026,"humidity":46},"datetime":0,"flags":1}
{"variant":7,"station":1,"sequence":0,"packed_bits":99,"packed_bytes":13,"battery":{"level":68,"charging":false},"rain":{"rate":14,"size":0},"temperature":20,"datetime":0,"flags":1}
{"variant":7,"station":10,"sequence":0,"packed_bits":99,"packed_bytes":13,"battery":{"level":90,"charging":false},"rain":{"rate":26,"size":4},"temperature":1,"datetime":0,"flags":1}
{"variant":2,"station":12,"sequence":0,"packed_bits":80,"packed_bytes":10,"battery":{"level":87,"charging":false},"soil_temp":12.75,"moisture":43,"depth":47,"flags":1}
{"variant":0,"station":15,"sequence":0,"packed_bits":247,"packed_bytes":31,"battery":{"level":71,"charging":false},"environment":{"temperature":13.25,"pressure":1025,"humidity":53},"wind":{"speed":6,"direction":3,"gust":7.5},"rain":{"rate":17,"size":4},"solar":{"irradiance":456,"ultraviolet":4},"clouds":2,"air_quality":139,"radiation":{"cpm":51,"dose":0.049999997019767761},"position":{"latitude":0.593342220386404,"longitude":0.18064142350203838},"datetime":0,"flags":1}
{"variant":2,"station":2,"sequence":0,"packed_bits":86,"packed_bytes":11,"battery":{"level":45,"charging":false},"link":{"rssi":-92,"snr":10},"soil_temp":14,"moisture":76,"depth":33,"flags":1}
{"variant":5,"station":6,"sequence":0,"packed_bits":102,"packed_bytes":13,"battery":{"level":65,"charging":false},"environment":{"temperature":23.5,"pressure":1036,"humidity":57},"datetime":0,"flags":1}
{"variant":0,"station":0,"sequence":8480,"packed_bits":76,"packed_bytes":10,"wind":{"speed":0,"direction":0,"gust":5.5},"solar":{"irradiance":985,"ultraviolet":10}}
{"variant":1,"station":8,"sequence":0,"packed_bits":171,"packed_bytes":22,"battery":{"level":42,"charging":false},"environment":{"temperature":27,"pressure":1009,"humidity":49},"air_quality":{"index":89,"pm":{"pm1":40,"pm25":55,"pm4":55,"pm10":35},"gas":{"voc":222,"nox":80}},"datetime":0,"flags":1}
{"variant":7,"station":10,"sequence":1,"packed_bits":67,"packed_bytes":9,"battery":{"level":90,"charging":false},"rain":{"rate":26,"size":4},"temperature":1.25}
{"variant":4,"station":13,"sequence":0,"packed_bits":182,"packed_bytes":23,"battery":{"level":84,"charging":false},"snow_depth":29,"environment":{"temperature":-6.25,"pressure":863,"humidity":59},"solar":{"irradiance":611,"ultraviolet":6},"position":{"latitude":0.611999667406053,"longitude":0.14999986588952652},"datetime":0,"flags":1}
{"variant":8,"station":3,"sequence":0,"packed_bits":130,"packed_bytes":17,"battery":{"level":90,"charging":false},"radiation":{"cpm":56,"dose":0.31000000238418579},"environment":{"temperature":13.25,"pressure":1010,"humidity":38},"datetime":0,"flags":1}
{"variant":5,"station":6,"sequence":1,"packed_bits":70,"packed_bytes":9,"battery":{"level":65,"charging":false},"environment":{"temperature":23.5,"pressure":1037,"humidity":58}}
{"variant":1,"station":16,"sequence":0,"packed_bits":177,"packed_bytes":23,"battery":{"level":58,"charging":false},"link":{"rssi":-64,"snr":-20},"environment":{"temperature":19,"pressure":1003,"humidity":31},"air_quality":{"index":40,"pm":{"pm1":50,"pm25":20,"pm4":85,"pm10":75},"gas":{"voc":180,"nox":104}},"datetime":0,"flags":1}
{"variant":7,"station":1,"sequence":1,"packed_bits":67,"packed_bytes":9,"battery":{"level":68,"charging":false},"rain":{"rate":14,"size":0},"temperature":19.75}
{"variant":2,"station":12,"sequence":1,"packed_bits":72,"packed_bytes":9,"battery":{"level":87,"charging":false},"soil_temp":12.75,"moisture":45,"depth":47}
{"variant":3,"station":4,"sequence":0,"packed_bits":97,"packed_bytes":13,"battery":{"level":74,"charging":false},"water_temp":10.75,"level":222,"datetime":0,"flags":1}
{"variant":0,"station":7,"sequence":0,"packed_bits":247,"packed_bytes":31,"battery":{"level":48,"charging":false},"environment":{"temperature":15,"pressure":1013,"humidity":28},"wind":{"speed":0.5,"direction":177,"gust":3},"rain":{"rate":0,"size":0},"solar":{"irradiance":0,"ultraviolet":1},"clouds":3,"air_quality":102,"radiation":{"cpm":44,"dose":0.19999998807907104},"position":{"latitude":0.593342220386404,"longitude":0.18064142350203838},"datetime":0,"flags":1}
{"variant":6,"station":14,"sequence":1,"packed_bits":82,"packed_bytes":11,"battery":{"level":52,"charging":false},"wind":{"speed":3.5,"direction":46,"gust":7.5},"solar":{"irradiance":169,"ultraviolet":10}}
{"variant":2,"station":2,"sequence":1,"packed_bits":72,"packed_bytes":9,"battery":{"level":45,"charging":false},"soil_temp":14,"moisture":77,"depth":33}
{"variant":7,"station":1,"sequence":2,"packed_bits":67,"packed_bytes":9,"battery":{"level":68,"charging":false},"rain":{"rate":14,"size":0},"temperature":20}
{"variant":0,"station":15,"sequence":1,"packed_bits":118,"packed_bytes":15,"battery":{"level":71,"charging":false},"environment":{"temperature":13.5,"pressure":1026,"humidity":53},"wind":{"speed":5,"direction":352,"gust":8},"rain":{"rate":17,"size":4},"solar":{"irradiance":484,"ultraviolet":3}}
{"variant":8,"station":11,"sequence":1,"packed_bits":98,"packed_bytes":13,"battery":{"level":84,"charging":false},"radiation":{"cpm":63,"dose":0.2199999988079071},"environment":{"temperature":17.5,"pressure":1027,"humidity":46}}
{"variant":5,"station":6,"sequence":2,"packed_bits":70,"packed_bytes":9,"battery":{"level":65,"charging":false},"environment":{"temperature":23.75,"pressure":1037,"humidity":56}}
{"variant":8,"station":3,"sequence":1,"packed_bits":98,"packed_bytes":13,"battery":{"level":90,"charging":false},"radiation":{"cpm":57,"dose":0.28999999165534973},"environment":{"temperature":13.25,"pressure":1011,"humidity":40}}
{"variant":2,"station":5,"sequence":1,"packed_bits":78,"packed_bytes":10,"battery":{"level":87,"charging":false},"link":{"rssi":-80,"snr":-20},"soil_temp":12.25,"moisture":25,"depth":58}
{"variant":1,"station":16,"sequence":1,"packed_bits":139,"packed_bytes":18,"battery":{"level":58,"charging":false},"environment":{"temperature":19,"pressure":1003,"humidity":33},"air_quality":{"index":34,"pm":{"pm1":45,"pm25":20,"pm4":90,"pm10":70},"gas":{"voc":178,"nox":98}}}
{"variant":7,"station":10,"sequence":2,"packed_bits":67,"packed_bytes":9,"battery":{"level":90,"charging":false},"rain":{"rate":19,"size":4},"temperature":1.25}
{"variant":1,"station":8,"sequence":1,"packed_bits":139,"packed_bytes":18,"battery":{"level":42,"charging":false},"environment":{"temperature":27.25,"pressure":1010,"humidity":47},"air_quality":{"index":90,"pm":{"pm1":45,"pm25":50,"pm4":60,"pm10":40},"gas":{"voc":220,"nox":72}}}
{"variant":1,"station":9,"sequence":1,"packed_bits":139,"packed_bytes":18,"battery":{"level":48,"charging":false},"environment":{"temperature":18.75,"pressure":995,"humidity":34},"air_quality":{"index":72,"pm":{"pm1":0,"pm25":5,"pm4":35,"pm10":40},"gas":{"voc":180,"nox":96}}}
{"variant":0,"station":7,"sequence":1,"packed_bits":118,"packed_bytes":15,"battery":{"level":48,"charging":false},"environment":{"temperature":15.25,"pressure":1012,"humidity":25},"wind":{"speed":0,"direction":181,"gust":1},"rain":{"rate":0,"size":0},"solar":{"irradiance":2,"ultraviolet":0}}
{"variant":2,"station":2,"sequence":2,"packed_bits":72,"packed_bytes":9,"battery":{"level":45,"charging":false},"soil_temp":14,"moisture":77,"depth":33}
{"variant":2,"station":12,"sequence":2,"packed_bits":72,"packed_bytes":9,"battery":{"level":87,"charging":false},"soil_temp":12.75,"moisture":43,"depth":47}
{"variant":4,"station":13,"sequence":1,"packed_bits":94,"packed_bytes":12,"battery":{"level":84,"charging":false},"snow_depth":29,"environment":{"temperature":-6.5,"pressure":862,"humidity":59},"solar":{"irradiance":599,"ultraviolet":7}}
{"variant":3,"station":4,"sequence":1,"packed_bits":65,"packed_bytes":9,"battery":{"level":74,"charging":false},"water_temp":10.75,"level":221}
{"variant":8,"station":11,"sequence":2,"packed_bits":98,"packed_bytes":13,"battery":{"level":84,"charging":false},"radiation":{"cpm":59,"dose":0.22999998927116394},"environment":{"temperature":17.5,"pressure":1027,"humidity":46}}
{"variant":8,"station":3,"sequence":2,"packed_bits":98,"packed_bytes":13,"battery":{"level":90,"charging":false},"radiation":{"cpm":57,"dose":0.26999998092651367},"environment":{"temperature":13.25,"pressure":1010,"humidity":41}}
{"variant":5,"station":6,"sequence":3,"packed_bits":70,"packed_bytes":9,"battery":{"level":65,"charging":false},"environment":{"temperature":23.75,"pressure":1038,"humidity":57}}
{"variant":6,"station":14,"sequence":2,"packed_bits":82,"packed_bytes":11,"battery":{"level":52,"charging":false},"wind":{"speed":4.5,"direction":30,"gust":7.5},"solar":{"irradiance":144,"ultraviolet":10}}
{"variant":1,"station":9,"sequence":2,"packed_bits":139,"packed_bytes":18,"battery":{"level":48,"charging":false},"environment":{"temperature":18.5,"pressure":994,"humidity":34},"air_quality":{"index":77,"pm":{"pm1":0,"pm25":5,"pm4":30,"pm10":45},"gas":{"voc":174,"nox":102}}}
{"variant":7,"station":1,"sequence":3,"packed_bits":73,"packed_bytes":10,"battery":{"level":68,"charging":false},"link":{"rssi":-80,"snr":0},"rain":{"rate":14,"size":0},"temperature":20.25}
{"variant":2,"station":5,"sequence":2,"packed_bits":78,"packed_bytes":10,"battery":{"level":87,"charging":false},"link":{"rssi":-84,"snr":-10},"soil_temp":12.25,"moisture":25,"depth":58}
{"variant":0,"station":15,"sequence":2,"packed_bits":124,"packed_bytes":16,"battery":{"level":71,"charging":false},"link":{"rssi":-60,"snr":-10},"environment":{"temperature":13.25,"pressure":1025,"humidity":56},"wind":{"speed":5.5,"direction":346,"gust":9.5},"rain":{"rate":17,"size":4},"solar":{"irradiance":473,"ultraviolet":3}}
{"variant":8,"station":11,"sequence":3,"packed_bits":98,"packed_bytes":13,"battery":{"level":84,"charging":false},"radiation":{"cpm":54,"dose":0.25},"environment":{"temperature":17.5,"pressure":1026,"humidity":44}}
{"variant":2,"station":12,"sequence":3,"packed_bits":72,"packed_bytes":9,"battery":{"level":87,"charging":false},"soil_temp":12.75,"moisture":44,"depth":47}
{"variant":5,"station":6,"sequence":4,"packed_bits":70,"packed_bytes":9,"battery":{"level":65,"charging":false},"environment":{"temperature":23.75,"pressure":1039,"humidity":58}}
{"variant":7,"station":10,"sequence":3,"packed_bits":67,"packed_bytes":9,"battery":{"level":90,"charging":false},"rain":{"rate":19,"size":4},"temperature":1}
{"variant":3,"station":4,"sequence":2,"packed_bits":65,"packed_bytes":9,"battery":{"level":74,"charging":false},"water_temp":10.75,"level":222}
{"variant":0,"station":7,"sequence":2,"packed_bits":124,"packed_bytes":16,"battery":{"level":48,"charging":false},"link":{"rssi":-80,"snr":0},"environment":{"temperature":15.5,"pressure":1011,"humidity":24},"wind":{"speed":0,"direction":195,"gust":3},"rain":{"rate":0,"size":0},"solar":{"irradiance":0,"ultraviolet":0}}
{"variant":1,"station":16,"sequence":2,"packed_bits":139,"packed_bytes":18,"battery":{"level":58,"charging":false},"environment":{"temperature":19.25,"pressure":1002,"humidity":35},"air_quality":{"index":41,"pm":{"pm1":45,"pm25":20,"pm4":90,"pm10":75},"gas":{"voc":176,"nox":94}}}
{"variant":2,"station":2,"sequence":3,"packed_bits":72,"packed_bytes":9,"battery":{"level":45,"charging":false},"soil_temp":14,"moisture":77,"depth":33}
{"variant":4,"station":13,"sequence":2,"packed_bits":94,"packed_bytes":12,"battery":{"level":84,"charging":false},"snow_depth":28,"environment":{"temperature":-6.25,"pressure":861,"humidity":60},"solar":{"irradiance":585,"ultraviolet":8}}
{"variant":7,"station":10,"sequence":4,"packed_bits":67,"packed_bytes":9,"battery":{"level":90,"charging":false},"rain":{"rate":19,"size":4},"temperature":1}
{"variant":2,"station":5,"sequence":3,"packed_bits":78,"packed_bytes":10,"battery":{"level":87,"charging":false},"link":{"rssi":-100,"snr":-10},"soil_temp":12.25,"moisture":26,"depth":58}
{"variant":8,"station":3,"sequence":3,"packed_bits":98,"packed_bytes":13,"battery":{"level":90,"charging":false},"radiation":{"cpm":52,"dose":0.25999999046325684},"environment":{"temperature":13,"pressure":1010,"humidity":40}}
{"variant":1,"station":9,"sequence":3,"packed_bits":139,"packed_bytes":18,"battery":{"level":48,"charging":false},"environment":{"temperature":18.5,"pressure":994,"humidity":35},"air_quality":{"index":70,"pm":{"pm1":0,"pm25":5,"pm4":35,"pm10":40},"gas":{"voc":176,"nox":96}}}
{"variant":8,"station":11,"sequence":4,"packed_bits":98,"packed_bytes":13,"battery":{"level":84,"charging":false},"radiation":{"cpm":54,"dose":0.22999998927116394},"environment":{"temperature":17.5,"pressure":1025,"humidity":42}}
```
