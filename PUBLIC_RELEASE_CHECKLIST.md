# PUBLIC-v1.0 RELEASE CHECKLIST

Project:

```
NM-CYD-C5-Muse2
```

Release:

```
PUBLIC-v1.0
v1.0.0
```

## 1. SOURCE CODE

* [ ] All intended source files are present.
* [ ] No placeholder implementation accidentally replaced real code.
* [ ] No debug-only source is unintentionally included.
* [ ] No generated source contains private information.

## 2. SECRETS

* [ ] No passwords.
* [ ] No API keys.
* [ ] No Wi-Fi credentials.
* [ ] No private signing keys.
* [ ] No private certificates.
* [ ] No cloud credentials.
* [ ] No production secrets.

## 3. DEVICE PRIVACY

* [ ] Private Muse device name removed or intentionally disclosed.
* [ ] BLE MAC addresses removed unless intentionally public.
* [ ] Device serial numbers removed unless intentionally public.
* [ ] Private device identifiers removed.
* [ ] Private calibration profiles removed.

## 4. EEG DATA

* [ ] No private EEG recordings.
* [ ] No personal EEG datasets.
* [ ] No private user information.
* [ ] No private calibration data.
* [ ] Captured raw BLE data reviewed before publication.

## 5. THIRD-PARTY SOFTWARE

* [ ] ESP-IDF licensing reviewed.
* [ ] FreeRTOS licensing reviewed.
* [ ] NimBLE licensing reviewed.
* [ ] Every additional library identified.
* [ ] Third-party copyright notices preserved.
* [ ] Third-party licenses preserved where required.
* [ ] No third-party code is accidentally placed under the project license.

## 6. TRADEMARKS

* [ ] Muse trademark ownership is not claimed.
* [ ] ESP32/Espressif trademark ownership is not claimed.
* [ ] Other third-party trademarks are not claimed.

## 7. EMPLOYER / CLIENT / INSTITUTIONAL IP

* [ ] No confidential employer code.
* [ ] No confidential client code.
* [ ] No unpublished company documentation.
* [ ] No laboratory confidential material.
* [ ] No university-owned material without authorization.
* [ ] Employment/contractual IP obligations reviewed.

## 8. SECURITY

* [ ] Secret scan completed.
* [ ] Git history scanned.
* [ ] SECURITY.md reviewed.
* [ ] Debug credentials removed.
* [ ] Production credentials separated from development credentials.

## 9. BUILD

* [ ] Clean build succeeds.
* [ ] Correct ESP32-C5 target selected.
* [ ] ESP-IDF version documented.
* [ ] No unnecessary build artifacts committed.
* [ ] Firmware tested on intended hardware.

## 10. MUSE / MU-03 VALIDATION

* [ ] BLE connection verified.
* [ ] EEG notifications verified.
* [ ] TP9 verified.
* [ ] AF7 verified.
* [ ] AF8 verified.
* [ ] TP10 verified.
* [ ] Raw packet captures reviewed.
* [ ] Packet length verified.
* [ ] Packet decoding verified against the target firmware revision.

## 11. RELEASE

* [ ] LICENSE present.
* [ ] COPYRIGHT present.
* [ ] SECURITY.md present.
* [ ] THIRD_PARTY_NOTICES.md present.
* [ ] README.md present.
* [ ] CHANGELOG.md present.
* [ ] VERSION present.
* [ ] .gitignore reviewed.
* [ ] PUBLIC_RELEASE_CHECKLIST.md completed.
* [ ] Git tag `v1.0.0` created.

## 12. FINAL RULE

Do not publish the repository until every file has been reviewed for
credentials, private data, third-party licensing, and intellectual-property
ownership.

