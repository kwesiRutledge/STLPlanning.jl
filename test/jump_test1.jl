"""
jump_test.jl
Description:
    A reminder of some of the julia algorithms which make integer programming work.
    JuMP is julia's optimization interface.
"""

using Test
using JuMP, Gurobi

@testset "1: Gurobi quick tests" begin
   
    # Create a Gurobi model
    model = Model(Gurobi.Optimizer)

    # Boiler Plate Optimization from Introduction to JuMP
    @variable(model, x >= 0)
    @variable(model, 0 <= y <= 3)
    @objective(model, Min, 12x + 20y)
    @constraint(model, c1, 6x + 8y >= 100)
    @constraint(model, c2, 7x + 12y >= 120)
    print(model)
    optimize!(model)
    @show termination_status(model)
    @show primal_status(model)
    @show dual_status(model)
    @show objective_value(model)
    @show value(x)
    @show value(y)
    @show shadow_price(c1)
    @show shadow_price(c2)

    @test value(x) == 15.0

end