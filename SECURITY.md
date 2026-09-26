# SECURITY.md — NM-CYD-C5-Muse2

## Reporting a security issue

Do not publish sensitive vulnerability details, credentials, private keys,
device identifiers, or exploit material in a public GitHub issue.

For sensitive security reports, use the private contact method published by
the project owner.

A useful security report should include:

* affected version or Git tag;
* affected file or component;
* reproducible steps;
* expected behavior;
* observed behavior;
* security impact;
* mitigation already tested, if applicable.

## NEVER COMMIT

Never commit the following information:

* passwords;
* API keys;
* private signing keys;
* Wi-Fi credentials;
* production certificates;
* cloud credentials;
* device-specific secrets;
* private calibration data;
* private EEG captures;
* private user data;
* employer confidential information;
* client confidential information;
* unpublished proprietary documentation.

## EMBEDDED SECURITY

For production hardware, evaluate and configure security controls appropriate
to the deployment, including:

* Secure Boot;
* Flash Encryption;
* encrypted NVS for sensitive persistent data;
* signed OTA firmware;
* secure OTA transport;
* debug/JTAG lockdown;
* unique production credentials;
* separate development and production signing keys.

These are security recommendations.

PUBLIC-v1.0 does NOT claim that all of these security mechanisms are enabled.

## BLE / MUSE SECURITY

This project contains device-specific BLE discovery and GATT handling.

Do not publish private BLE device names, addresses, identifiers, or other
device-specific information unless intentional.

"Muse" and related product names and trademarks belong to their respective
owners.

NM-CYD-C5-Muse2 is an independent engineering/research project and is not
represented as an official Muse product.

## EEG DATA

EEG and calibration data may be sensitive.

Captured EEG data, calibration profiles, personal information, and device
identifiers should be treated as private unless there is a clear reason and
authorization to publish them.

## PACKET FORMAT

The DSP implementation contains assumptions about the MU-03 EEG packet format.

The exact packet structure should be validated against the target hardware and
firmware revision before relying on decoded EEG values for production,
scientific, medical, or safety-critical purposes.

## SECURITY FIXES

Security fixes should normally be released under a new version tag.

Do not silently rewrite an already-published release in a way that hides the
security history.

## PUBLIC RELEASE

Before pushing this project to a public GitHub repository, run secret scanning
against both:

1. the current working tree; and
2. Git history.

Also inspect device identifiers, logs, configuration files, calibration data,
and third-party licensing.

