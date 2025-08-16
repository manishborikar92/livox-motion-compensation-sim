# Essential Hardware Components List
## Livox Mid-70 Train-Mounted LiDAR Scanning System

---

## 🎯 System Overview

**Train-Mounted LiDAR Scanning System**
- **Primary Purpose**: Full train scanning with roof-mounted LiDAR sensor
- **Target Accuracy**: <2cm relative accuracy for train geometry mapping
- **Application**: Train inspection, clearance verification, maintenance assessment
- **System Type**: Mobile scanning platform with vibration-resistant design
- **Operating Environment**: Railway conditions with high vibration, weather exposure

---

## 📦 Core LiDAR System Components

### **1. Livox Mid-70 LiDAR Sensor**
| Component | Specification | Vendor | Part Number | Unit Cost |
|-----------|---------------|--------|-------------|-----------|
| **Mid-70 Sensor** | 70.4° FOV, 90m range, 5cm min range | Livox/DJI | Mid-70 | $1,399 |
| **Livox Converter 2.0** | Power/Ethernet interface | Livox/DJI | Converter 2.0 | Included |
| **Standard Cable Set** | 5m Power + Ethernet | Livox/DJI | Standard | Included |
| **Mounting Bracket** | 1/4"-20 thread mount | Livox/DJI | Standard | Included |

**Package Contents Verified:**
- ✅ Mid-70 LiDAR unit (760g, 88×69×102mm)
- ✅ Converter 2.0 (88g, 74×52×23mm)  
- ✅ 5m power/Ethernet cable
- ✅ Mounting hardware and documentation
- ✅ IP67 protection (sensor unit)

**Key Specifications:**
- **Detection Range**: 90m @ 10% / 130m @ 20% reflectivity
- **Accuracy**: ±2cm (1σ @ 25m)
- **FOV**: 70.4° circular (zero blind spots)
- **Point Rate**: 100,000 points/second maximum
- **Built-in IMU**: 6-axis, 200Hz update rate
- **Latest Firmware**: 03.08.0000

---

## 🔌 Power Supply System (Railway Integration)

### **Train Power Integration (Required)**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Railway DC-DC Converter** | 110V/24V → 12V/15A, Railway certified | Vicor | DCM4623T15F43C5 | $485 |
| **EMI Filter** | Railway-grade, EN 50121-3-2 | Schaffner | FN2200-25/06 | $145 |
| **Surge Protection** | Railway transient protection | Phoenix Contact | PT 2X2-24DC-ST | $89 |
| **Power Distribution** | Railway-certified fused block | Phoenix Contact | PTFIX 6X2.5-NS35 | $65 |
| **Backup Battery** | 12V/20Ah LiFePO4, vibration resistant | Battle Born | BB10012 | $299 |
| **Battery Management** | BMS with railway certification | Victron | SmartBMS CL 12-100 | $195 |

---

## 🛰️ Position and Motion Tracking (Required for Train Applications)

### **High-Precision GNSS/INS System (Essential)**
| Component | Specification | Vendor | Part Number | Unit Cost |
|-----------|---------------|--------|-------------|-----------|
| **GNSS/INS Receiver** | 1cm + 1ppm RTK, 0.008° attitude, Railway certified | NovAtel | PwrPak7-E1 | $15,000 |
| **GNSS Antenna** | Multi-frequency L1/L2/L5, Railway mounting | NovAtel | GPS-704X | $800 |
| **Antenna Cable** | Low-loss, 20m, railway-grade | NovAtel | 20ARINC | $295 |
| **Lightning Protection** | GNSS antenna surge protector | PolyPhaser | GPS-50NF | $125 |

**Critical Specifications for Train Mounting:**
- Position: 1cm + 1ppm (RTK), 2m (autonomous)
- Attitude: 0.008° roll/pitch, 0.015° heading
- Update Rate: 200Hz position, 400Hz IMU
- Vibration: MIL-STD-810G compliant
- Temperature: -40°C to +75°C
- Shock: 40G, 11ms duration

---

*This specification represents the essential hardware requirements for train-mounted Livox Mid-70 LiDAR scanning systems. All components have been selected for railway compliance and operational reliability in demanding train environments.*
