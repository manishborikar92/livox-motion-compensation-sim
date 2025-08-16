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

## 💻 Computing Platform (Railway-Hardened)

### **Industrial Computing System (Required)**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Rugged Computer** | i7-11700, 32GB, EN 50155 certified | Neousys | RTC-1000 | $3,200 |
| **Storage** | 4TB NVMe SSD, industrial grade | Samsung | PM1733 4TB | $899 |
| **Backup Storage** | 2TB removable SSD | Samsung | T7 Shield 2TB | $199 |
| **Network Interface** | Dual Gigabit Ethernet, M12 connectors | Advantech | PCM-26D2CA | $245 |

**Railway Certification Requirements:**
- EN 50155 (Railway Electronics)
- EN 50121 (EMC for Railway)
- IEC 61373 (Shock and Vibration)
- Operating Temperature: -25°C to +70°C
- Vibration: 5G RMS, 5-150Hz
- Shock: 30G, 11ms duration

---

## 🔧 Train Roof Mounting System (Critical Components)

### **Primary Mounting Assembly**
| Component | Specification | Vendor | Part Number | Unit Cost |
|-----------|---------------|--------|-------------|-----------|
| **Roof Mount Base** | 800×400×15mm 6061-T6 Al, railway certified | Custom/Railcar | RCM-800-15 | $450 |
| **LiDAR Gimbal Mount** | 2-axis stabilized, ±15° compensation | Custom/Precision | PGM-2A-15 | $1,200 |
| **GNSS Antenna Mast** | 1.5m carbon fiber, lightning protected | Comrod | CF-1500-LP | $350 |
| **Vibration Isolators** | Railway-grade, 80 Shore A, M12 thread | Lord | J-9613-1 | $85/ea ×6 |
| **Shock Mounts** | Active damping, 50G protection | Enidine | OEM-1.5M8 | $125/ea ×4 |

### **Environmental Protection (Railway Standards)**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Main Enclosure** | IP67, 600×400×300mm, EN 50155 | Rittal | CP 6536.500 | $485 |
| **LiDAR Housing** | IP67, heated window, anti-ice | Custom/Protective | LH-M70-HW | $650 |
| **Cable Glands** | IP68, M25 thread, railway certified | Lapp | SKINTOP STR-M25 | $18/ea ×12 |
| **Pressure Relief** | Breathable membrane | Gore | PMF200 | $45 |
| **Heating System** | 12V, 150W, thermostat controlled | Omega | AHP-3751 | $189 |

### **Installation Hardware**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Roof Penetration Kit** | Waterproof roof entry system | Dicor | 552TPO-25 | $125 |
| **Structural Bolts** | M12×80mm, Grade 8.8, stainless | McMaster-Carr | 92855A542 | $8/ea ×16 |
| **Sealant** | Railway-approved structural adhesive | 3M | 5200FC | $35 |
| **Grounding Kit** | Lightning protection system | PolyPhaser | DSX-NF | $95 |

---

## 🔌 Railway-Grade Cables and Connectivity

### **Primary System Cables (Railway Certified)**
| Component | Specification | Vendor | Part Number | Unit Cost |
|-----------|---------------|--------|-------------|-----------|
| **Ethernet Cable** | Cat6A, 30m, railway-grade, M12 connectors | Belden | 1305F-30M | $245 |
| **Power Cable** | 10AWG, 30m, railway certified, flame retardant | Alpha Wire | 3080-30 | $165 |
| **GNSS Coax** | RG-58, 20m, low-loss, railway grade | Times Microwave | LMR-195-20M | $89 |
| **PPS Timing Cable** | BNC-BNC, 15m, 50Ω, railway certified | Pomona | 2249-C-180 | $95 |

### **Connectors and Protection**
| Component | Specification | Vendor | Part Number | Unit Cost |
|-----------|---------------|--------|-------------|-----------|
| **M12 Ethernet Connectors** | IP67, 8-pin, railway certified | Phoenix Contact | SACC-M12MS-8CON | $35/ea ×4 |
| **Power Connectors** | IP67, 4-pin, 30A rating | Amphenol | C016-30D004-100-12 | $45/ea ×2 |
| **BNC Connectors** | 50Ω, weatherproof, surge protected | Amphenol | 31-221-RFX | $25/ea ×4 |
| **Cable Strain Reliefs** | M25 thread, IP68 rated | Lapp | SKINTOP ST-M25 | $12/ea ×8 |

### **Cable Management**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Cable Tray** | Stainless steel, perforated, 100mm wide | Unistrut | P1000T-10PG | $125 |
| **Cable Ties** | UV resistant, railway approved | Panduit | PLT2S-M0 | $25/pack |
| **Protective Conduit** | Flexible, flame retardant, 25mm | Adaptaflex | AFLEX25 | $8/meter ×10m |

---

## 🛠️ Installation and Calibration Tools (Required)

### **Installation Tools**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Railway Drill Set** | Cordless, metal drilling, railway certified | Hilti | SF 6H-A22 | $485 |
| **Torque Wrench Set** | 10-200 Nm range, calibrated | Snap-on | TECH3FR250 | $325 |
| **Multimeter** | True RMS, railway applications | Fluke | 87V-MAX | $525 |
| **Alignment Laser** | Precision mounting alignment | Leica | Lino L2P5 | $450 |

### **Calibration Equipment (Essential)**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Calibration Targets** | Reflective spheres, 6" diameter | 3D Target | RS-6-10 | $650 |
| **Precision Level** | 0.01° accuracy, digital | Wyler | BlueLEVEL | $1,200 |
| **Distance Meter** | Laser rangefinder, ±1mm accuracy | Leica | DISTO D810 | $650 |

---

## 📊 System Monitoring and Data Management

### **Real-Time Monitoring (Required)**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Industrial Display** | 10" touchscreen, railway certified | Advantech | TPC-1051WP | $1,250 |
| **System Health Monitor** | Multi-parameter monitoring | National Instruments | cDAQ-9178 | $1,495 |
| **Data Logger** | High-speed, railway-grade | Dewetron | DEWE3-A4 | $2,850 |

### **Data Storage and Processing**
| Component | Specification | Vendor | Model | Unit Cost |
|-----------|---------------|--------|-------|-----------|
| **Primary Storage** | 8TB NVMe RAID-1, industrial | Samsung | PM1733 4TB ×2 | $1,798 |
| **Backup Storage** | 4TB portable SSD, encrypted | Samsung | T7 Shield 4TB | $399 |
| **Point Cloud Software** | Professional railway analysis | Bentley | MicroStation CONNECT | $2,500 |

---

## 📦 Complete Train-Mounted LiDAR System

### **Essential System Configuration**
**Application**: Train roof-mounted LiDAR scanning for full train geometry capture

| Category | Components | Subtotal |
|----------|------------|----------|
| **LiDAR System** | Mid-70 + Converter 2.0 + Railway housing | $2,049 |
| **GNSS/INS** | NovAtel PwrPak7-E1 + Antenna + Lightning protection | $16,220 |
| **Computing** | Railway-certified rugged computer + storage | $4,298 |
| **Power System** | Railway DC-DC converter + EMI filter + backup battery | $1,278 |
| **Mounting System** | Roof mount + gimbal + vibration isolation | $2,210 |
| **Environmental Protection** | IP67 enclosures + heating + cable management | $1,392 |
| **Cables & Connectivity** | Railway-grade cables + M12 connectors | $696 |
| **Installation Tools** | Railway-certified tools + calibration equipment | $3,780 |
| **Monitoring & Storage** | Industrial display + data logging + software | $8,492 |
| **Total System Cost** | **Train-Mounted LiDAR System** | **$40,415** |

### **Critical Spare Parts (Recommended)**
| Component | Quantity | Purpose | Unit Cost |
|-----------|----------|---------|-----------|
| **Spare LiDAR Unit** | 1 | Critical backup | $1,399 |
| **Spare Cables** | Complete set | Field replacements | $696 |
| **Backup Storage** | 4TB encrypted SSD | Data redundancy | $399 |
| **Spare Fuses/Breakers** | Assorted ratings | Power protection | $125 |
| **Total Spares Cost** | **Recommended Spares** | **$2,619** |

---

*This specification represents the essential hardware requirements for train-mounted Livox Mid-70 LiDAR scanning systems. All components have been selected for railway compliance and operational reliability in demanding train environments.*
