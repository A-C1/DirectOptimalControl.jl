````julia
using JuMP
model = Model()

x1 = @variable(model, base_name = "x1")
x2 = x1 + 1
x3::AffExpr = x1

x4 = x1 + sin(x1) - sin(x1)
````

Works: x1 is of type VariableRef

````julia
v1 = [sin(x1), x1]
````

Works: x2 is of type AffExpr

````julia
v2 = [sin(x1), x2]
````

Does not work: x3 is also of type AffExpr

````julia
v3 = [sin(x1), x4]
````

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

