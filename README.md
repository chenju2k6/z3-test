### Build with docker (recommended)

```
docker build --tag z3-test  --file Dockerfile .
```

### Test

To test single branch constraints

```
docker run -it z3-test /bin/bash
cd build
./z3test ../test
```

To test nested branch constraints

```
docker run -it z3-test /bin/bash
cd build
./z3ntest ../test_nested
```
