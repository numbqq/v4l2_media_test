# Build

```
$ g++ v4l2_test_raw.cpp -o v4l2_test_raw  -Wall -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast -pthread -fPIE -lispaml -ldl -lmediaAPI -ltuning -lm -fPIC -D_FORTIFY_SOURCE=2 -O3
```

# Test

```
$ ./v4l2_test_raw -p 0 -n 100 -m /dev/media0
```

It will capture 100 frames to file `/tmp/dst_mif_0.yuv`.

# Preview the file

```
$ ffplay -f rawvideo -pixel_format nv21 -video_size 3840x2160 /tmp/dst_mif_0.yuv
```
