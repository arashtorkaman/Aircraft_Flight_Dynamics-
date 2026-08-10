# Phase 3 Data

This folder receives the Phase 1/2 longitudinal plant exported by:

```matlab
import_phase1_model(A,B,'U0',U0,'h0',h0,'Kq',Kq)
```

Expected generated file:

```text
phase1_longitudinal_model.mat
```

Do not replace the original aircraft model with matrices reconstructed only from the pole locations. Pole locations do not uniquely determine the state-space realization or input matrix.
