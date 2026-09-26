# NM-CYD-C5-Muse2

**PUBLIC-v1.0 — Version 1.0.0**

ESP32-C5 + Muse 2 EEG engineering and research project using ESP-IDF and
NimBLE.

## PROJECT

NM-CYD-C5-Muse2 is an embedded ESP32-C5 project designed to communicate
with Muse 2 / MU-03 EEG hardware over Bluetooth Low Energy.

The project is intended for engineering, experimentation, research, and
embedded-system development.

## PLATFORM

* MCU: ESP32-C5
* Framework: ESP-IDF
* ESP-IDF development version: 5.5.2
* Bluetooth: NimBLE
* RTOS: FreeRTOS through ESP-IDF
* EEG channels:

  * TP9
  * AF7
  * AF8
  * TP10

## EEG PROCESSING

The project includes a separate EEG DSP processing path.

The intended processing pipeline includes:

1. BLE EEG notification reception
2. packet transfer outside the NimBLE callback
3. channel identification
4. packet buffering
5. EEG sample decoding
6. windowing
7. FFT
8. one-sided PSD
9. frequency-band analysis

## IMPORTANT PACKET-FORMAT NOTICE

The current DSP implementation assumes a Muse-style 20-byte EEG packet
containing:

* 2 sequence bytes
* 18 bytes containing twelve packed 12-bit samples

This assumption MUST be validated against the exact MU-03 hardware and
firmware revision being used.

Do not treat decoded values as authoritative until the packet format has been
verified against real captures from the target device.

## PUBLIC DEVICE-NAME PROTECTION

The public source should not contain a private device-specific identifier.

Use a placeholder such as:

```
#define MUSE_NAME "Muse-xxxx"
```

Before flashing your private development hardware, replace the placeholder
locally with the actual device name.

Do not commit a private device name, BLE address, serial number, or other
unique identifier unless you intentionally want that information public.

## OWNERSHIP

Original project-specific material is copyrighted by:

```
Copyright (c) 2026 Shibily VM.
All rights reserved.
```

See:

* `LICENSE`
* `COPYRIGHT`

## THIRD-PARTY OWNERSHIP

This project does NOT claim ownership of:

* ESP-IDF
* FreeRTOS
* NimBLE
* ESP32-C5 hardware
* Muse hardware
* Muse firmware
* Muse trademarks
* Muse logos
* compiler/toolchain components
* third-party libraries
* third-party fonts
* third-party graphics
* third-party datasets
* other third-party intellectual property

Those materials remain owned by their respective rights holders.

See:

```
THIRD_PARTY_NOTICES.md
```

## SECURITY

Before building a production device, review:

```
SECURITY.md
```

In particular, evaluate:

* Secure Boot
* Flash Encryption
* encrypted NVS
* signed OTA
* secure OTA transport
* JTAG/debug security
* production credentials
* secret management

## DATA PRIVACY

EEG data and calibration data should be treated as potentially sensitive.

Do not commit:

* EEG recordings
* private calibration profiles
* personal information
* device identifiers
* private BLE addresses
* passwords
* API keys
* private certificates
* private signing keys

## ACKNOWLEDGEMENTS

Developer:

```
Shibily VM
```

Background / acknowledgements supplied by the author:

```
Chettuva, Thrissur, Kerala, India
BITS Pilani — Student
Dubai Technologies
Geepas, Dubai
Orion Hardware Development Lab
```

Mentors acknowledged:

```
Midhulaj PJ Sir
Swathi Miss
```

## STATUS

PUBLIC-v1.0 / 1.0.0

This is an engineering/research source release.

Before making the complete repository public, perform a repository-wide audit
for:

* credentials;
* private device identifiers;
* EEG data;
* calibration data;
* employer/client confidential material;
* institutional IP;
* third-party source code;
* third-party licenses;
* generated files;
* build artifacts.

## MEDICAL DISCLAIMER

NM-CYD-C5-Muse2 is an engineering/research project.

It is not a medical device.

It is not intended to diagnose, treat, cure, or prevent disease.

It should not be used as a safety-critical or medical decision-making system.

