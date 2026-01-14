# ROS 2 Humble Geliştirme Ortamı

Bu proje ROS 2 Humble ile uygulama geliştirmek için Docker tabanlı bir geliştirme ortamı sağlar.

## Gereksinimler

- Docker
- Docker Compose
- X11 (GUI uygulamaları için - Linux sistemlerde)
- Git

## İlk Kurulum (Yeni Bilgisayar İçin)

### 1. Projeyi klonlayın:
```bash
git clone https://github.com/Hyildirim8/ros2_humble_docker_template.git
cd ros2_humble_docker_template
```

### 2. Ortam değişkenlerini ayarlayın ve Docker imajını oluşturun:
```bash
export UID=$(id -u)
export GID=$(id -g)
export USER=$(whoami)
docker compose build
```

**Not:** Windows kullanıcıları için PowerShell'de:
```powershell
$env:UID=1000
$env:GID=1000
$env:USER=$env:USERNAME
docker-compose build
```

### 3. Konteyneri başlatın:
```bash
docker compose up -d
```

### 4. Konteynere bağlanın:
```bash
docker exec -it ros2-humble-dev bash
```

### 5. Workspace zaten mevcut, doğrudan kullanabilirsiniz:
```bash
cd /workspace
# Mevcut paketleri derleyin
colcon build
source install/setup.bash
# Mevcut node'u çalıştırın
ros2 run simple_py_pkg sample_node
```

### 5. Yeni paket oluşturma örneği

```bash
cd /workspace/src
ros2 pkg create --build-type ament_python my_package
# veya C++ için:
ros2 pkg create --build-type ament_cmake my_cpp_package
```

### 6. Workspace'i derleyin

```bash
cd /workspace
colcon build
source install/setup.bash
```

## Günlük Kullanım

### Konteyneri durdurma

```bash
docker compose down
```

### Konteyneri yeniden başlatma

```bash
export UID=$(id -u) && export GID=$(id -g) && export USER=$(whoami)
docker compose up -d
```

### Konteynere bağlanma

```bash
docker exec -it ros2-humble-dev bash
```

## Yararlı ROS 2 Komutları

### ROS 2 komutları kontrol etme

```bash
ros2 --help
```

### Mevcut node'ları listeleme

```bash
ros2 node list
```

### Topic'leri listeleme

```bash
ros2 topic list
```

### Paket listesi

```bash
ros2 pkg list
```

### Node çalıştırma

```bash
ros2 run <paket_adı> <node_adı>
# Örnek:
ros2 run simple_py_pkg sample_node
```

## GUI Uygulamaları

Docker konteyneri X11 forwarding ile yapılandırılmıştır. RViz, Gazebo gibi GUI uygulamalarını çalıştırabilirsiniz:

```bash
rviz2
```

**Not (Linux):** İlk seferde GUI erişim hatası alırsanız:

```bash
xhost +local:docker
```

## Sorun Giderme

### "Package not found" Hatası

Eğer `ros2 run` komutu paketi bulamazsa:

```bash
cd /workspace
colcon build
source install/setup.bash
```

### "Permission denied" veya Dosya Kaydetme Sorunu

Eğer workspace içinde dosya kaydedemiyorsanız:

```bash
# Host sistemde:
sudo chown -R $(whoami):$(whoami) ./workspace/
```

### Konteyner Kullanıcı Sorunu

"I have no name!" hatası alırsanız, konteyneri yeniden build edin:

```bash
docker-compose down
docker rmi ros2-humble-dev:latest
export UID=$(id -u) && export GID=$(id -g) && export USER=$(whoami)
docker-compose build --no-cache
docker-compose up -d
```

## Proje Yapısı

```
ros2_humble_docker_template/
├── docker-compose.yml      # Docker Compose yapılandırması
├── Dockerfile              # Docker image tanımı
├── README.md              # Bu dosya
├── start.sh               # Başlatma scripti (opsiyonel)
└── workspace/             # ROS 2 workspace (paylaşımlı)
    ├── src/               # Kaynak kodlar
    │   └── simple_py_pkg/ # Örnek Python paketi
    ├── build/             # Build çıktıları
    ├── install/           # Kurulum dosyaları
    └── log/               # Log dosyaları
```

## Not

- `workspace` klasörü ana sisteminizle paylaşılır, bu nedenle kodlarınız kalıcıdır.
- Container içinde yapılan değişiklikler `workspace` dışında container durdurulduğunda kaybolur.
- Her yeni terminal oturumunda ortam değişkenlerini (`UID`, `GID`, `USER`) export etmeyi unutmayın.
- Farklı bilgisayarlarda kullanmak için sadece projeyi klonlayın ve yukarıdaki adımları takip edin.

colcon build --packages-select simple_py_pkg --symlink-install