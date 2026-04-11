# bubul

It is a particle simulator aimed at explaining thermodynamics from microscopic simulation. You need the [demo2d package](https://github.com/HerveFrezza-Buet/demo2d)

# install

```{bash}
git clone https://github.com/HerveFrezza-Buet/bubul
mkdir -p bubul/build
cd bubul/build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
sudo make install
cd ../..
```
# run examples

```{bash}
bubul-example-001-001-collision
bubul-example-001-002-gas SQUARE 1000
```


