# Sensör Füzyonu ve Lokalizasyon Kullanarak LiDAR Tabanlı Otonom Navigasyon

**Geliştirici:** Zeynep Selin Kaptan (Öğrenci No: 21406601043])
**Kurum:** Bursa Teknik Üniversitesi - Mekatronik Mühendisliği Bölümü

Bu proje, Mobil Robotlar dersi teknik gereksinimleri çerçevesinde, diferansiyel sürüşlü (non-holonomic) otonom bir mobil robotun 2B simülasyon ortamında Yapay Potansiyel Alan (APF) algoritması ile yönlendirilmesini ve Genişletilmiş Kalman Filtresi (EKF) kullanılarak lokalizasyon doğruluğunun iyileştirilmesini içermektedir.

## Sistem Özellikleri ve Mimari
- **Navigasyon Modeli:** Diferansiyel robot kinematiği kısıtları altında Yapay Potansiyel Alan (APF) reaktif algoritması ile dinamik engelden kaçınma.
- **Sensör Modellemesi:** Gaussian gürültü karakteristiğine sahip 3 metre mesafe sınırlamalı (thresholding) 2B LiDAR ve tekerlek enkoderi odometrisi simülasyonu.
- **Sensör Füzyonu:** Kümülatif odometri kayma hatalarını önlemek amacıyla tasarlanmış Genişletilmiş Kalman Filtresi (EKF) tabanlı durum tahmini.

## Proje Dizini Yapısı
- `main.py`: Simülasyon döngüsünü, EKF algoritmasını koşturan ve 4 alt analiz çıktısını grafik haline getiren ana Python kodu.
- `Rapor.pdf`: Proje senaryosunu, metodolojiyi, RMSE hata değerlendirmelerini ve akademik atıfları barındıran teslim raporu.
- `grafik_sonuclar.png`: Simülasyon sonucunda üretilen 4'lü performans analiz çıktısı.

## Çalıştırma Talimatı
Projenin yerel bilgisayarınızda simüle edilebilmesi için sisteminizde Python 3.x ortamının ve gerekli matematiksel kütüphanelerin bulunması gerekmektedir.

Gerekli paketleri terminal üzerinden kurmak için:
```bash
pip install numpy matplotlib
