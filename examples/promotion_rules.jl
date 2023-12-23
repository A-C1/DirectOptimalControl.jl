using JuMP
model = Model()

x1 = @variable(model, base_name = "x1")
x2 = x1 + 1
x3::AffExpr = x1 

# Works: x1 is of type VariableRef
v1 = [sin(x[1]), x1]
# Works: x2 is of type AffExpr
v2 = [sin(x[1]), x2]
# Does not work: x3 is also of type AffExpr
v3 = [sin(x1), x3]