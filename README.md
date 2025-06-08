# 🚓 TurtleBot4 Multi-Robot Park Patroll

**Autonomous illegal parking detection system using TurtleBot4 robots**, developed as part of the **Doosan Robotics Bootcamp 2025**.

---

## 📌 Overview

This project demonstrates a **multi-robot surveillance system** using two TurtleBot4 robots. The robots autonomously patrol an area, detect parked vehicles, recognize license plates, and verify them against a database. If a violation is detected, the robot triggers a warning signal (🔊 *beep-boop alert*).

---

## 🛠️ Tech Stack

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/TurtleBot4-robot-brightgreen?logo=raspberrypi" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/YOLOv8-CV-red?logo=opencv" />
  <img src="https://img.shields.io/badge/EasyOCR-OCR-orange?logo=openai" />
  <img src="https://img.shields.io/badge/SQLite-database-lightgrey?logo=sqlite" />
  <img src="https://img.shields.io/badge/MySQL-database-blue?logo=mysql" />
  <img src="https://img.shields.io/badge/FastAPI-backend-green?logo=fastapi" />
  <img src="https://img.shields.io/badge/Docker-container-2496ED?logo=docker" />
  <img src="https://img.shields.io/badge/Nav2-navigation-critical?logo=mapbox" />
</p>



---

## 🎯 Use Case Scenario

1. A vehicle enters a **blind zone** and is detected using **YOLO-based vision**.
2. The nearest TurtleBot4 robot executes **Nav2 navigation** to approach the vehicle.
3. The robot performs **local patrol** and identifies any **parked vehicle**.
4. The license plate number and current position are sent to a **central system controller**.
5. The controller queries a **local database** to verify the license plate.
6. If the vehicle is unauthorized → 🚨 **illegal parking alert** is triggered.
7. The alert is visualized on a **GUI dashboard** and stored in a **monitoring database**, displaying the **vehicle number and zone information**.
8. The system operates in **two zones**, each covered by a **dedicated TurtleBot4 robot** (multi-robot architecture).

---

## 🎥 Demo Video

[▶️ Watch the full demo on YouTube](https://your_demo_video_link_here)

---

## 🖼️ License Plate Recognition

<div align="center">
  <img src="https://your_license_plate_recognition_image_link_here.jpg" width="60%">
  <p><i>Example: Successfully detected and matched license plate</i></p>
</div>

---

## 🔁 System Architecture

* `robot0`: Patrols Zone 1, checks vehicle presence, detects license plate, and raises alert.
* `robot1`: Operates in Zone 2, performs same tasks independently.
* **Shared Services**:

  * License plate OCR module
  * Vehicle number database service

---

## 👥 Contributors

Thanks to these wonderful people who have contributed to this project:

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/weedmo">
        <img src="https://github.com/weedmo.png" width="100px;" alt="weedmo"/><br />
        <sub><b>weedmo</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/yujin114">
        <img src="https://github.com/yujin114.png" width="100px;" alt="yujin114"/><br />
        <sub><b>yujin114</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/ethica-alt">
        <img src="https://github.com/ethica-alt.png" width="100px;" alt="ethica-alt"/><br />
        <sub><b>ethica-alt</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/jsbae-RL">
        <img src="https://github.com/weedmo.png" width="100px;" alt="jsbae-RL"/><br />
        <sub><b>jsbae-RL</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/weedmo">
        <img src="https://github.com/weedmo.png" width="100px;" alt="weedmo"/><br />
        <sub><b>weedmo</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/weedmo">
        <img src="https://github.com/weedmo.png" width="100px;" alt="weedmo"/><br />
        <sub><b>weedmo</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/weedmo">
        <img src="https://github.com/weedmo.png" width="100px;" alt="weedmo"/><br />
        <sub><b>weedmo</b></sub>
      </a>
    </td>
  </tr>
</table>


---

