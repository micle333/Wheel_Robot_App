import subprocess

def start_rtsp_stream():
    command = [
        "ffmpeg",
        "-f", "avfoundation",
        "-pix_fmt", "yuyv422",
        "-video_size", "640x480",
        "-framerate", "30",
        "-i", "0:0",  # 0:0 для встроенной камеры Mac
        "-ac", "2",  # 2 канала для стерео
        "-vf", "format=yuyv422",
        "-vcodec", "libx264",
        "-maxrate", "2000k",
        "-bufsize", "2000k",
        "-acodec", "aac",
        "-ar", "44100",
        "-b:a", "128k",
        "-f", "rtp_mpegts",
        "udp://192.168.51.137:9988"  # Передача через UDP
    ]
    
    # Запускаем процесс FFmpeg
    subprocess.run(command)

if __name__ == "__main__":
    print("Starting RTP stream on udp://192.168.51.101:9988")
    start_rtsp_stream()
