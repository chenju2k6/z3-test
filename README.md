### Build with docker (recommended)

```
docker build --tag z3-test  --file Dockerfile .
```

### Test

```
docker run -it z3-test /bin/bash
cd build
./z3test ../test
```
