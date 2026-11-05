# Security Policy

## Supported versions

Only the latest released version receives security fixes.

| Version | Supported |
|---------|-----------|
| 1.x     | yes       |

## Reporting a vulnerability

Please report vulnerabilities privately through GitHub:
[Security advisories](https://github.com/bekirhankarakurt/uav-mission-planner/security/advisories/new).
Do not open a public issue for security problems.

Include the affected version, a description of the problem, and a minimal input file or command that
reproduces it. You can expect an acknowledgement within a few days.

## Scope and safety notes

- The planner parses untrusted file formats (SRTM `.hgt`, GeoJSON, WPL, QGroundControl `.plan`).
  Crashes, hangs or memory errors triggered by malformed input are in scope.
- This software produces flight plans. It is **not** certified flight software: always review a plan
  in your ground control station, verify it against current airspace rules and local terrain data,
  and fly with appropriate failsafes.
