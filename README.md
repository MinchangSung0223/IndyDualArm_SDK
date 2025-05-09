# IndyDualArm_SDK

```bash
mkdir build
cd build
cmake ..
make -j32
sudo make install
```

## Example
```bash
cd exmamples
sudo ldconfig
mkdir build
cd build
camke ..
make -j32
./test_dualarm
```

![image](https://github.com/user-attachments/assets/b5e37a49-8f09-48ab-a368-5c2c09615bd1)
# MATLAB Code Generation
```bash
cd slx
./start_codegeneration.sh
cd ../codegen
python3 codegen_setup.py
cd ../build
cmake ..
make -j32
```
