import pytube

url = "https://www.youtube.com/watch?v=4mGLSbYLmDE"

yt = pytube.YouTube(url)
video = yt.streams.get_lowest_resolution()
video.download()
