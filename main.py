import numpy as np
import matplotlib.pyplot as plt
import math

# --- SİSTEM VE SENARYO PARAMETRELERİ ---
DT = 0.1  # Örnekleme zamanı (saniye)
MAX_SURE = 60.0  # Simülasyon bitiş süresi
HEDEF_KOORDİNAT = np.array([10.0, 10.0])
ROBOT_BASLANGIC = np.array([0.0, 0.0, 0.0])  # x, y, theta

# Robot Kinematik Sınırları
HIZ_MAX = 1.2  # m/s
ACISAL_HIZ_MAX = np.deg2rad(45.0)  # rad/s

# APF Navigasyon Parametreleri
K_CEKIM = 1.5
ETA_ITME = 30.0
LIDAR_ALGI_SINIRI = 3.0  # Metre

# Sensör Gürültü Kovaryans Matrisleri
Q_ODOMETRI = np.diag([0.15, 0.15, np.deg2rad(5.0)]) ** 2  # Enkoder hatası
R_LIDAR = np.diag([0.4, 0.4]) ** 2  # LiDAR konumlandırma hatası

# Senaryo Gereği Eklenen 12 Adet Statik Engel Konumu
ORTAM_ENGELLERI = np.array([
    [2, 2], [2, 3], [3, 5], [5, 4], [5, 5], [5, 6],
    [7, 7], [8, 6], [7, 2], [8, 3], [1, 7], [9, 8]
])


def diferansiyel_robot_model(durum, v, w, delta_t):
    """ Non-holonomic mobil robot kinematik denklemleri """
    yeni_durum = np.copy(durum)
    yeni_durum[0] += v * math.cos(durum[2]) * delta_t
    yeni_durum[1] += v * math.sin(durum[2]) * delta_t
    yeni_durum[2] += w * delta_t
    return yeni_durum


def apf_kontrol_komutu(durum):
    """ Yapay Potansiyel Alan hesaplaması ile hız komutları üretimi """
    dx_h = HEDEF_KOORDİNAT[0] - durum[0]
    dy_h = HEDEF_KOORDİNAT[1] - durum[1]

    fx_toplam = K_CEKIM * dx_h
    fy_toplam = K_CEKIM * dy_h

    algilanan_engeller = []

    for e in ORTAM_ENGELLERI:
        dx_e = durum[0] - e[0]
        dy_e = durum[1] - e[1]
        mesafe = math.hypot(dx_e, dy_e)

        # LiDAR Mesafe Eşikleme
        if mesafe < LIDAR_ALGI_SINIRI:
            algilanan_engeller.append(e)
            if mesafe < 0.2: mesafe = 0.2  # Sıfıra bölme koruması

            itme_kuvveti = ETA_ITME * (1.0 / mesafe - 1.0 / LIDAR_ALGI_SINIRI) / (mesafe ** 2)
            fx_toplam += itme_kuvveti * (dx_e / mesafe)
            fy_toplam += itme_kuvveti * (dy_e / mesafe)

    hedef_theta = math.atan2(fy_toplam, fx_toplam)
    theta_hatasi = hedef_theta - durum[2]

    # Açı normalizasyonu [-pi, pi]
    theta_hatasi = (theta_hatasi + math.pi) % (2 * math.pi) - math.pi

    w_komut = 2.5 * theta_hatasi
    v_komut = 0.5 * math.hypot(fx_toplam, fy_toplam)

    # Doyum limitleri
    v_komut = np.clip(v_komut, -HIZ_MAX, HIZ_MAX)
    w_komut = np.clip(w_komut, -ACISAL_HIZ_MAX, ACISAL_HIZ_MAX)

    return v_komut, w_komut, np.array(algilanan_engeller)


def ekf_sensor_fuzyonu(x_eski, P_eski, z_olcum, v, w, delta_t):
    """ Genişletilmiş Kalman Filtresi algoritması ardışık adımları """
    # 1. Adım: Durum Öngörüsü (Predict)
    x_pred = np.zeros((3, 1))
    x_pred[0, 0] = x_eski[0, 0] + v * math.cos(x_eski[2, 0]) * delta_t
    x_pred[1, 0] = x_eski[1, 0] + v * math.sin(x_eski[2, 0]) * delta_t
    x_pred[2, 0] = x_eski[2, 0] + w * delta_t

    # Sistemin Jacobian Matrisi (jF)
    th = x_eski[2, 0]
    jF = np.array([
        [1.0, 0.0, -delta_t * v * math.sin(th)],
        [0.0, 1.0, delta_t * v * math.cos(th)],
        [0.0, 0.0, 1.0]
    ])

    P_pred = jF @ P_eski @ jF.T + Q_ODOMETRI

    # 2. Adım: Durum Güncellemesi (Update)
    jH = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ])

    z_pred = jH @ x_pred
    inovasyon = z_olcum - z_pred
    S = jH @ P_pred @ jH.T + R_LIDAR
    K_kazanc = P_pred @ jH.T @ np.linalg.inv(S)

    x_yeni = x_pred + K_kazanc @ inovasyon
    P_yeni = (np.eye(3) - K_kazanc @ jH) @ P_pred

    return x_yeni, P_yeni


def main():
    zaman = 0.0

    # Başlangıç vektör tanımlamaları
    x_gercek = np.array([ROBOT_BASLANGIC]).T
    x_dr = np.array([ROBOT_BASLANGIC]).T
    x_ekf = np.array([ROBOT_BASLANGIC]).T
    P_matrisi = np.eye(3)

    # Veri kayıt dizileri
    log_gercek = x_gercek
    log_dr = x_dr
    log_ekf = x_ekf
    log_z = x_gercek[0:2]
    son_lidar_noktalari = []

    while zaman <= MAX_SURE:
        zaman += DT

        # APF üzerinden hızları hesapla
        v_ref, w_ref, engeller_lidar = apf_kontrol_komutu(x_gercek.flatten())
        if len(engeller_lidar) > 0:
            son_lidar_noktalari = engeller_lidar

        # Gerçek robot modelini yürüt
        x_gercek = diferansiyel_robot_model(x_gercek.flatten(), v_ref, w_ref, DT).reshape(-1, 1)

        # Gürültülü sensör ölçüm simülasyonları
        z_lidar_konum = np.array([
            [x_gercek[0, 0] + np.random.randn() * np.sqrt(R_LIDAR[0, 0])],
            [x_gercek[1, 0] + np.random.randn() * np.sqrt(R_LIDAR[1, 1])]
        ])

        v_enkoder = v_ref + np.random.randn() * np.sqrt(Q_ODOMETRI[0, 0])
        w_enkoder = w_ref + np.random.randn() * np.sqrt(Q_ODOMETRI[2, 2])

        # Sadece odometri tahmini (Dead Reckoning)
        x_dr = diferansiyel_robot_model(x_dr.flatten(), v_enkoder, w_enkoder, DT).reshape(-1, 1)

        # Genişletilmiş Kalman Filtresi döngüsü
        x_ekf, P_matrisi = ekf_sensor_fuzyonu(x_ekf, P_matrisi, z_lidar_konum, v_enkoder, w_enkoder, DT)

        # Geçmiş kayıt hafızası güncelleme
        log_gercek = np.hstack((log_gercek, x_gercek))
        log_dr = np.hstack((log_dr, x_dr))
        log_ekf = np.hstack((log_ekf, x_ekf))
        log_z = np.hstack((log_z, z_lidar_konum))

        # Hedef kontrolü
        if math.hypot(x_gercek[0, 0] - HEDEF_KOORDİNAT[0], x_gercek[1, 0] - HEDEF_KOORDİNAT[1]) < 0.4:
            print("Mobil robot hedef koordinatına güvenle ulaştı.")
            break

    # Hata metrik analizleri
    h_serisi_dr = np.sqrt((log_gercek[0, :] - log_dr[0, :]) ** 2 + (log_gercek[1, :] - log_dr[1, :]) ** 2)
    h_serisi_ekf = np.sqrt((log_gercek[0, :] - log_ekf[0, :]) ** 2 + (log_gercek[1, :] - log_ekf[1, :]) ** 2)

    rmse_dr_degeri = np.sqrt(np.mean(h_serisi_dr ** 2))
    rmse_ekf_degeri = np.sqrt(np.mean(h_serisi_ekf ** 2))

    # --- MATPLOTLIB GÖRSELLEŞTİRME MATRİSİ ---
    plt.figure(figsize=(15, 9))

    # Alt Grafik 1: Ortam ve Rota Planlama (Gereksinim 6.1 ve 6.2)
    plt.subplot(2, 2, 1)
    plt.plot(log_gercek[0, :], log_gercek[1, :], "-g", linewidth=2.5, label="Gerçek Rota")
    plt.plot(log_dr[0, :], log_dr[1, :], "--k", alpha=0.7, label="Dead Reckoning (Odometri)")
    plt.plot(log_ekf[0, :], log_ekf[1, :], "-r", linewidth=2, label="Sensör Füzyonu (EKF)")
    plt.scatter(ORTAM_ENGELLERI[:, 0], ORTAM_ENGELLERI[:, 1], c='black', marker='s', s=120, label="Depo Engelleri")
    plt.plot(ROBOT_BASLANGIC[0], ROBOT_BASLANGIC[1], "bo", markersize=9, label="Başlangıç (0,0)")
    plt.plot(HEDEF_KOORDİNAT[0], HEDEF_KOORDİNAT[1], "y*", markersize=16, label="Hedef (10,10)")
    plt.title("2B Harita Alanı Üzerinde Konum Rotaları")
    plt.xlabel("X Ekseni [m]");
    plt.ylabel("Y Ekseni [m]")
    plt.legend();
    plt.grid(True)

    # Alt Grafik 2: Sensör Ham Veri Dağılımı (Gereksinim 6.3)
    plt.subplot(2, 2, 2)
    plt.plot(log_z[0, :], log_z[1, :], ".c", alpha=0.3, label="Gürültülü Ham LiDAR Konum Verisi")
    plt.plot(log_ekf[0, :], log_ekf[1, :], "-r", linewidth=2, label="Filtrelenmiş EKF Rotası")
    if len(son_lidar_noktalari) > 0:
        plt.scatter(son_lidar_noktalari[:, 0], son_lidar_noktalari[:, 1], c='purple', marker='X', s=80,
                    label="Eşiklenen Son LiDAR Engeli")
    plt.title("Sensör Sinyal Analizi: Ham vs Filtrelenmiş")
    plt.xlabel("X Ekseni [m]");
    plt.ylabel("Y Ekseni [m]")
    plt.legend();
    plt.grid(True)

    # Alt Grafik 3: Zaman Serisi Doğrulaması (Gereksinim 6.4)
    zaman_ekseni = np.arange(log_gercek.shape[1]) * DT
    plt.subplot(2, 2, 3)
    plt.plot(zaman_ekseni, log_gercek[0, :], "-g", label="Gerçek Konum X")
    plt.plot(zaman_ekseni, log_ekf[0, :], "--r", label="EKF Tahmin X")
    plt.plot(zaman_ekseni, log_gercek[1, :], "-b", label="Gerçek Konum Y")
    plt.plot(zaman_ekseni, log_ekf[1, :], "--m", label="EKF Tahmin Y")
    plt.title("Zaman Serisinde X ve Y Eksen Analizi")
    plt.xlabel("Zaman [s]");
    plt.ylabel("Konum Ölçüsü [m]")
    plt.legend();
    plt.grid(True)

    # Alt Grafik 4: Kümülatif Hata Karşılaştırması (Gereksinim 6.5)
    plt.subplot(2, 2, 4)
    plt.plot(zaman_ekseni, h_serisi_dr, "-k", alpha=0.6, label=f"Dead Reckoning Sapması (RMSE: {rmse_dr_degeri:.2f}m)")
    plt.plot(zaman_ekseni, h_serisi_ekf, "-r", linewidth=2, label=f"EKF Füzyon Sapması (RMSE: {rmse_ekf_degeri:.2f}m)")
    plt.title("Zaman Boyunca Mutlak Konum Sapma Grafiği")
    plt.xlabel("Zaman [s]");
    plt.ylabel("Hata Mesafesi [m]")
    plt.legend();
    plt.grid(True)

    plt.tight_layout()
    # Görseli otomatik kaydetme bloğu
    plt.savefig("grafik_sonuclar.png", dpi=300)
    plt.show()


if __name__ == '__main__':
    main()