$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$out = Join-Path $PSScriptRoot 'out'
$dcMotor = Join-Path $root 'Drivers\DC_MOTOR'
$mpu6050 = Join-Path $root 'Drivers\MPU6050'

New-Item -ItemType Directory -Force $out | Out-Null

gcc -std=c11 -Wall -Wextra -Werror `
    -I $dcMotor `
    (Join-Path $PSScriptRoot 'test_motor.c') `
    (Join-Path $dcMotor 'motor.c') `
    -o (Join-Path $out 'test_motor.exe')

gcc -std=c11 -Wall -Wextra -Werror `
    -I $mpu6050 `
    (Join-Path $PSScriptRoot 'test_mpu6050.c') `
    (Join-Path $mpu6050 'mpu6050.c') `
    -o (Join-Path $out 'test_mpu6050.exe')

& (Join-Path $out 'test_motor.exe')
& (Join-Path $out 'test_mpu6050.exe')

Write-Host 'H_CAR module tests: PASS'
