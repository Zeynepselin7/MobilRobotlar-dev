# Sensör Füzyonu ve Lokalizasyon Kullanarak LiDAR Tabanlı Otonom Navigasyon
Zeynep Selin Kaptan 21406601043
Bu proje, Mobil Robotlar dersi kapsamında, sensör füzyonu (Genişletilmiş Kalman Filtresi) ve lokalizasyon algoritmaları kullanılarak, LiDAR donanımlı non-holonomic bir mobil robotun 2B ortamda otonom navigasyonunu gerçekleştirmek üzere geliştirilmiştir.

## Proje İçeriği
- `main.py`: Simülasyonu, navigasyonu (APF), sensör füzyonunu (EKF) ve görselleştirmeleri çalıştıran ana Python betiği.
- `Rapor.pdf`: Proje gereksinimlerini, yöntemleri, hata analizlerini (RMSE) ve sonuç grafiklerini içeren detaylı proje raporu.
- `grafik_sonuclar.png`: Kod çalıştırıldığında otomatik olarak üretilen ve tüm analizleri içeren görsel çıktı.

## Kurulum ve Çalıştırma Talimatları
Projeyi yerel ortamınızda çalıştırmak için Python 3.x yüklü olmalıdır. Gerekli kütüphaneleri kurmak için terminalde aşağıdaki komutu çalıştırınız:

```bash
pip install numpy matplotlib
