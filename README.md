# rp2040-rs

```
rustup target add thumbv6m-none-eabi
cargo build
cargo run --release
```

generate rgb565 rawfile using
```
ffmpeg -vcodec png -i image.png -vcodec rawvideo -f rawvideo -pix_fmt rgb565 image.raw
```

# LICENSE
GPL-3.0
