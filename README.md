# Monitoring-Kualitas-Udara
# Sistem Monitoring Kualitas Udara Real-time dengan Notifikasi Ambang Batas

Proyek ini adalah sistem pemantauan kualitas udara berbasis ESP32 yang mengukur berbagai parameter lingkungan seperti konsentrasi debu (PM2.5), suhu, kelembapan udara, kadar CO2, dan kadar NH3. Data ditampilkan secara real-time pada panel LED P10 dan juga dapat diintegrasikan dengan Arduino IoT Cloud untuk pemantauan jarak jauh. Fitur utama dari sistem ini adalah indikasi visual pada panel P10 ketika nilai sensor melebihi ambang batas yang ditentukan.

## Fitur Utama

* **Pemantauan Multi-Sensor**: Mengukur PM2.5, suhu, kelembapan, CO2, dan NH3.
* **Tampilan Real-time**: Menampilkan semua data sensor dan waktu saat ini pada panel LED P10 (128x16).
* **Indikasi Ambang Batas**: Ketika nilai sensor melebihi batas normal yang ditentukan, area sensor tersebut pada panel P10 akan di-highlight (teks menjadi hitam dengan latar belakang terang) sebagai peringatan visual.
* **Sinkronisasi Waktu**: Menggunakan modul Real-Time Clock (RTC) DS3231 untuk menjaga akurasi waktu.
* **Integrasi Arduino IoT Cloud**: Mengirimkan data sensor ke Arduino IoT Cloud untuk pemantauan dan analisis data jarak jauh (membutuhkan konfigurasi `Secrets.h` dan `thingProperties.h`).
* **Multitasking dengan FreeRTOS**: Menggunakan beberapa task untuk menangani pembacaan sensor, pembaruan tampilan P10, pembaruan RTC, dan komunikasi IoT secara efisien.

## Perangkat Keras yang Dibutuhkan

1.  **Mikrokontroler**: ESP32 Development Board.
2.  **Panel Display**: Panel LED P10 (ukuran 128x16 piksel, atau konfigurasi lain yang sesuai dengan library PxMatrix).
3.  **Sensor Suhu & Kelembapan**: DHT22 (atau DHT11 dengan sedikit penyesuaian).
4.  **Sensor Kualitas Udara**:
    * MQ-135: Untuk CO2 dan NH3 (membutuhkan kalibrasi yang sesuai).
    * DSM501A (atau sensor PM2.5 sejenis): Untuk konsentrasi debu PM2.5.
5.  **Modul Real-Time Clock**: DS3231 (untuk akurasi waktu yang lebih baik).
6.  **Kabel Jumper dan Breadboard (jika diperlukan)**.
7.  **Power Supply yang Sesuai**: Untuk ESP32 dan panel P10 (panel P10 bisa membutuhkan arus yang cukup besar).

## Perangkat Lunak & Library

1.  **Arduino IDE** atau **PlatformIO** untuk pengembangan.
2.  **ESP32 Core** untuk Arduino.
3.  **Library yang Digunakan**:
    * `PxMatrix` oleh 2dom (untuk kontrol panel P10).
    * `Adafruit GFX Library` (dependensi PxMatrix).
    * `RTClib` oleh Adafruit (untuk RTC DS3231).
    * `DHTesp` oleh beegee_tokyo (untuk sensor DHT pada ESP32).
    * `AverageValue` (library kustom atau pihak ketiga untuk menghaluskan pembacaan sensor).
    * `ArduinoIoTCloud` oleh Arduino.
    * `Arduino_ConnectionHandler` (dependensi ArduinoIoTCloud).
    * (Library lain yang mungkin menjadi dependensi dari library di atas).

## Konfigurasi Pin

Berikut adalah definisi pin yang digunakan dalam kode (sesuaikan jika Anda menggunakan konfigurasi berbeda):

* **Panel P10**:
    * `P_A`: GPIO 2
    * `P_B`: GPIO 16
    * `P_LAT`: GPIO 19
    * `P_OE`: GPIO 17
    * `SPI_MOSI`: GPIO 23 (referensi, dikelola PxMatrix)
    * `SPI_SCK`: GPIO 18 (referensi, dikelola PxMatrix)
* **Sensor DHT22**:
    * `PIN_DHT22`: GPIO 26
* **Sensor MQ-135**:
    * `PIN_MQ_A0`: GPIO 35 (pin ADC)
* **Sensor DSM501A**:
    * `PIN_DSM501`: GPIO 25

## Struktur Kode

Kode utama diorganisir menjadi beberapa bagian utama:

1.  **Konfigurasi Global & Inklusi Library**:
    * Definisi konfigurasi panel PxMatrix.
    * Inklusi semua library yang dibutuhkan.
    * Definisi pin untuk semua sensor dan panel.
    * Deklarasi objek global untuk panel, RTC, dan sensor.
    * Konstanta kalibrasi untuk sensor MQ-135 dan konfigurasi DSM501.
    * Struktur data `date_time_t` untuk waktu.
    * Variabel global untuk menyimpan data sensor, nilai ambang batas, dan status RTC.
    * Prototipe fungsi untuk semua task.

2.  **Kontrol Panel P10 (`P10_...` functions)**:
    * `P10_begin()`: Inisialisasi panel P10.
    * `P10_clear()`: Membersihkan tampilan panel.
    * `P10_show()`: Menampilkan buffer ke panel (jika double buffering aktif).
    * `P10_bottom(temp, humi, pm25)`: Menggambar data suhu, kelembapan, jam, dan PM2.5 di bagian bawah panel.
    * `P10_top(co2, nh3)`: Menggambar data CO2 dan NH3 di bagian atas panel.
    * `P10_fill(x, y, blocks)`: Fungsi helper untuk mengisi area tertentu di panel dengan warna solid (digunakan untuk highlight saat nilai melebihi ambang batas).
    * **Logika Ambang Batas**: Fungsi `P10_bottom` dan `P10_top` kini memiliki logika untuk mengubah warna teks dan latar belakang area sensor jika nilainya melebihi `temp_thres`, `humi_thres`, `pm25_thres`, `co2_thres`, atau `nh3_thres`.

3.  **Kontrol RTC (`RTC_...` functions & `display_jam`)**:
    * `RTC_begin()`: Inisialisasi modul DS3231.
    * `RTC_update_time_and_get_temp()`: Membaca waktu dan suhu dari RTC, memperbarui variabel `currentTime`.
    * `display_jam()`: Menampilkan waktu dari `currentTime` ke panel P10.

4.  **Kontrol Sensor (DHT22, MQ-135, DSM501A)**:
    * `DHT22_begin()`, `get_humidity_dht22()`, `get_temperature_dht22()`: Inisialisasi dan pembacaan data dari sensor DHT22.
    * `MQ135_begin()`, `get_nh3_mq135()`, `get_co2_mq135()`: Inisialisasi dan pembacaan data (dengan rata-rata) dari sensor MQ-135.
    * `DSM501A_begin()`, `DSM501A_read()`: Inisialisasi dan pembacaan data dari sensor DSM501A.

5.  **Integrasi Arduino IoT Cloud**:
    * `Secrets.h`: Menyimpan kredensial WiFi dan Device Key Arduino IoT Cloud (perlu dibuat sendiri).
    * `thingProperties.h`: Dihasilkan oleh Arduino IoT Cloud, berisi definisi properti Cloud (misalnya `nH3`, `pM25`, `tEMPERATURE`, `cO2`, `hUMIDITY`).
    * Fungsi callback kosong (misalnya `onCO2Change()`) sebagai placeholder untuk aksi jika ada perubahan dari Cloud.
    * `initProperties()`: Menginisialisasi properti Cloud.
    * Variabel Cloud (misalnya `tEMPERATURE`, `hUMIDITY`) diupdate di `TaskSensor`.

6.  **Manajemen Task FreeRTOS & `setup()`/`loop()`**:
    * `setup()`:
        * Inisialisasi Serial Monitor.
        * Inisialisasi semua perangkat keras (P10, RTC, sensor).
        * Inisialisasi properti dan koneksi Arduino IoT Cloud.
        * Membuat semua task FreeRTOS.
    * `loop()`: Dibiarkan minimal, hanya berisi `vTaskDelay` karena semua pekerjaan utama dilakukan oleh task.
    * **Tasks**:
        * `TaskUpdateIoT(void* pvParameters)`: Bertanggung jawab untuk memanggil `ArduinoCloud.update()` secara berkala (setiap 15 detik) untuk menjaga koneksi dan sinkronisasi dengan Cloud.
        * `TaskUpdateRTC(void* pvParameters)`: Bertanggung jawab memperbarui variabel global `currentTime` dari modul RTC setiap detik. (Catatan: Pada kode terbaru Anda, *critical section* (`rtcMux`) tidak digunakan di task ini saat mengakses `currentTime`, yang berpotensi menimbulkan *race condition*. Sebaiknya ditambahkan kembali untuk keamanan data).
        * `TaskSensor(void* pvParameters)`: Membaca semua data sensor (DHT22, MQ-135, DSM501A) secara periodik (setiap 1 detik, namun didominasi oleh `samplingTime` DSM501A). Juga memperbarui variabel global dan variabel Cloud.
        * `TaskDisplay(void* pvParameters)`: Memperbarui tampilan pada panel P10 (~10 kali per detik). Mengecek perubahan data sensor atau waktu, dan jika ada, menggambar ulang seluruh panel dengan data terbaru, termasuk logika highlight ambang batas. (Catatan: Akses `currentTime` di task ini juga sebaiknya menggunakan *critical section* yang sama dengan `TaskUpdateRTC`).
        * `TaskRefresh(void* pvParameters)`: Berjalan dengan prioritas tertinggi, bertanggung jawab untuk me-refresh/scan panel P10 secara kontinu (`display.display()`) agar tampilan stabil dan tidak berkedip.

## Penyiapan Proyek

1.  **Persiapkan Perangkat Keras**: Rakit semua komponen sesuai dengan skema koneksi dan definisi pin di atas.
2.  **Instalasi Arduino IDE & ESP32 Core**: Pastikan Arduino IDE Anda terinstal dan ESP32 core sudah ditambahkan melalui Boards Manager.
3.  **Instalasi Library**: Instal semua library yang disebutkan di atas melalui Arduino Library Manager:
    * `PxMatrix`
    * `Adafruit GFX Library`
    * `RTClib`
    * `DHTesp`
    * `AverageValue` (jika ini library pihak ketiga, pastikan Anda menambahkannya dengan benar)
    * `ArduinoIoTCloud`
    * `Arduino_ConnectionHandler`
4.  **Konfigurasi Arduino IoT Cloud**:
    * Buat "Thing" baru di [Arduino IoT Cloud](https://create.arduino.cc/iot/).
    * Tambahkan properti (variabel) yang sesuai dengan yang ada di kode (`nH3`, `pM25`, `tEMPERATURE`, `cO2`, `hUMIDITY`). Pastikan tipe datanya cocok (misalnya `float`, `CloudTemperatureSensor`, `int`, `CloudRelativeHumidity`).
    * Setelah membuat Thing dan variabel, Anda akan mendapatkan `Device ID` dan `Secret Key`.
    * Download file `thingProperties.h`.
5.  **Buat dan Konfigurasi `Secrets.h`**:
    * Buat file baru bernama `Secrets.h` di dalam folder sketsa Anda.
    * Isi file tersebut dengan kredensial WiFi dan Device Key Anda:
        ```cpp
        #define SECRET_SSID "NAMA_WIFI_ANDA"
        #define SECRET_OPTIONAL_PASS "PASSWORD_WIFI_ANDA"
        #define SECRET_DEVICE_KEY "DEVICE_KEY_DARI_IOT_CLOUD"
        ```
6.  **Sesuaikan Parameter (Jika Perlu)**:
    * Nilai kalibrasi sensor MQ-135 (`nilaiRo_CO2`, `Ro_NH3`, dll.) mungkin perlu disesuaikan berdasarkan kalibrasi sensor spesifik Anda.
    * Nilai ambang batas sensor (`temp_thres`, `humi_thres`, dll.) bisa diubah sesuai kebutuhan.
    * `samplingTime` untuk DSM501A.
7.  **Upload Kode**: Pilih board ESP32 yang sesuai dan port yang benar, lalu upload sketsa.

## Cara Kerja Sistem

1.  Saat ESP32 dinyalakan, fungsi `setup()` menginisialisasi semua perangkat keras, koneksi ke Arduino IoT Cloud, dan membuat lima task FreeRTOS yang berjalan secara konkuren.
2.  `TaskUpdateRTC` secara kontinu (setiap detik) membaca waktu dari modul DS3231 dan memperbarui variabel global `currentTime`.
3.  `TaskSensor` secara periodik (siklusnya didominasi oleh 15 detik `samplingTime` DSM501A) membaca data dari semua sensor (DHT22, MQ-135, DSM501A). Data ini disimpan dalam variabel global (`temp`, `humi`, `co2`, `nh3`, `pm25`) dan juga disalin ke variabel yang terhubung dengan Arduino IoT Cloud (`tEMPERATURE`, `hUMIDITY`, dll.).
4.  `TaskDisplay` berjalan kira-kira 10 kali per detik. Ia membandingkan data sensor global dan waktu saat ini dengan data yang terakhir ditampilkan. Jika ada perubahan, seluruh panel P10 dibersihkan dan digambar ulang. Fungsi `P10_bottom` dan `P10_top` akan menampilkan data dan juga menerapkan logika *highlight* jika nilai sensor melewati ambang batas yang telah ditentukan. `P10_show()` kemudian menampilkan hasilnya.
5.  `TaskRefresh` berjalan dengan prioritas tertinggi dan sangat cepat, terus menerus memanggil `display.display()` untuk memastikan panel P10 selalu di-scan dan menampilkan gambar yang stabil.
6.  `TaskUpdateIoT` secara berkala (setiap 15 detik) memanggil `ArduinoCloud.update()` untuk mengirim data terbaru ke cloud dan menerima perintah jika ada.

## Potensi Pengembangan / Masalah yang Diketahui

* **Blocking pada `DSM501A_read()`**: Fungsi pembacaan sensor PM2.5 saat ini bersifat blocking selama `samplingTime` (15 detik). Ini menyebabkan `TaskSensor` hanya memperbarui semua bacaan sensornya (termasuk DHT, MQ135) setiap ~16 detik. Untuk pembaruan sensor yang lebih cepat, `DSM501A_read()` perlu diubah menjadi non-blocking (menggunakan pendekatan state machine).
* **Sinkronisasi Akses `currentTime`**: Seperti yang telah didiskusikan, akses ke variabel global `currentTime` dari `TaskUpdateRTC` (penulis) dan `TaskDisplay`/`display_jam` (pembaca) saat ini tidak menggunakan *critical section* (`rtcMux` dideklarasikan tapi tidak dipakai dalam kode terakhir Anda untuk proteksi). Ini berpotensi menyebabkan *race condition*. Sangat disarankan untuk mengimplementasikan kembali proteksi ini untuk integritas data waktu.
* **Kalibrasi Sensor MQ-135**: Akurasi sensor MQ-135 sangat bergantung pada kalibrasi yang tepat (penentuan nilai `Ro`) untuk gas target di lingkungan yang diketahui. Nilai `Ro` yang ada di kode mungkin perlu penyesuaian.
* **Efisiensi Redraw P10**: Saat ini, `TaskDisplay` melakukan clear dan redraw penuh jika ada perubahan. Untuk sistem yang lebih kompleks atau tampilan yang lebih besar, optimasi dengan hanya menggambar ulang bagian yang berubah (*partial updates*) bisa dipertimbangkan, meskipun akan menambah kompleksitas.

---

Semoga dokumentasi ini bermanfaat untuk proyek Anda di GitHub!
