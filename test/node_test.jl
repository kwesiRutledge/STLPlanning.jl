"""
node_test.jl
Description:
    Testing the constructors and the functions used for Node
    type objects.
"""

using Test

include("../src/PWLPlan.jl")

@testset "1. ConjunctionNode Tests" begin
    # Create a Conjunction Node
    cn1 = ConjunctionNode([1.0,2.0])

    @test cn1.Constraints == []
    @test cn1.Children == Vector{Float64}([1.0,2.0])
end

@testset "2. noIntersection Tests" begin
    # Create a Conjunction Node
    a = 1.0
    b = 2.0
    c = 3.0
    d = 4.0

    noIntersectionNode1 = noIntersection(a,b,c,d)
    
    @test noIntersectionNode1.Children[1] < c - b
    @test noIntersectionNode1.Children[2] < d - a

end