# Установка и Запуск Gazebo Simulation на Windows

Эти инструкции помогут вам установить и запустить Gazebo Simulation на Windows с использованием Windows Subsystem for Linux (WSL), Ubuntu, Docker и x11-xserver-utils.

## Шаги установки

1. **Установить Windows Subsystem for Linux (WSL)**

    Следуйте официальным инструкциям Microsoft для [установки WSL](https://docs.microsoft.com/ru-ru/windows/wsl/install).

2. **Установить Ubuntu на WSL (20.04)**

    Вы можете установить Ubuntu через Microsoft Store. Выберите Ubuntu 20.04.

3. **Установить Docker на Windows**

    Скачайте и установите Docker для Windows с официального сайта [Docker](https://www.docker.com/products/docker-desktop).

4. **Проверить интеграцию WSL в Docker**

    Убедитесь, что WSL интегрирован с Docker в настройках Docker Desktop.

5. **Установить x11-xserver-utils**

    Откройте Ubuntu в WSL и выполните команду:

    ```
    sudo apt-get install x11-xserver-utils
    ```

6. **Склонировать репозиторий в WSL**

    Используйте команду `git clone` для клонирования вашего репозитория с проектом Gazebo.

7. В WSL перейдите в директорию проекта и запустите:
```
./build_docker.sh
```
8. После успешной сборки контейнера запустите:
```
./run.bash
```
9. Для входа в запущенный контейнер используйте:
```
./into.bash
```
Теперь вы можете выполнять работу внутри контейнера. Например можно запустить Gazebo Simulator
```
gazebo
```
Выполните необходимые операции внутри контейнера.
По завершении работы в контейнере, для его остановки выполните:
```
./stop.bash
```
