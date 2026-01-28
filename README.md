# KUBRA NUR TIRYAKI
# orbslam3-droidcam-noetic
ROS Noetic üzerinde ORB-SLAM3 (Monocular) + DroidCam (telefon kamerası) ile çalışan örnek kurulum/çalıştırma akışı.

Bu repo; telefon kamerasını MJPEG üzerinden ROS topic’ine çevirme, kamera kalibrasyonu alma, ORB-SLAM3 monocular çalıştırma ve monocular “ölçek belirsizliği” için ölçüm + pose ölçekleme araçlarını içerir.

## Hızlı Başlangıç (Çalıştırma Akışı - Özet)
1) roscore
2) DroidCam → ROS image_raw/camera_info
3) (Bir kere) camera_calibration ile intrinsics çıkar ve YAML’e yaz
4) ORB-SLAM3 monocular çalıştır
5) RViz ile 3D iki nokta tıklayıp SLAM mesafesi ölç
6) scale = real/slam
7) pose’u scale ile yayınla (`/orb_slam3/camera_pose_scaled`)

---

## İçindekiler
- 1) Önkoşullar
- 2) Build
- 3) Çalıştırma Akışı (Özet)
- 4) Kamera Kalibrasyonu
- 5) ORB-SLAM3 Çalıştırma (Monocular)
- 6) Görüntü İzleme
- 7) 3D Mesafe Ölçümü (RViz ile iki nokta arası)
- 8) Pose Ölçekleme (Monocular scale düzeltme)
- Scriptler (bu repoda eklenenler)
- Sık karşılaşılan sorunlar

---

## 1) Önkoşullar
- Ubuntu 20.04 + ROS Noetic
- catkin_tools
- ORB-SLAM3 bağımlılıkları (repo içindeki `Dependencies.md` / mevcut kurulum)
- DroidCam (telefon app) veya MJPEG stream verebilen benzeri bir uygulama

Gerekli ROS paketleri:
```bash
sudo apt update
sudo apt install -y python3-catkin-tools \
  ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-camera-info-manager ros-noetic-camera-calibration \
  ros-noetic-rqt-image-view ros-noetic-rviz
