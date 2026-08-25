### Usage

```
cd tests

# run the entire test suite
./test_suite.py

# don't perform the initial make stage
./test_suite.py --nomake

# run the fast smoke subset
python smoke/run_smoke.py --nomake

# run the benchmark world set and collect performance logs
python benchmarks/run_benchmarks.py --nomake

# run tests individually
./test_suite.py api/worlds/gps.omniworld parser/worlds/empty_value.omniworld

# run tests individually without the test suite framework (the test_suite_supervisor returns directly)
../webots api/worlds/gps.omniworld
```

### Missing tests

- gyro
- propeller
- led
- position sensors
- charger
- physics friction test
- physics damping test
- joints
