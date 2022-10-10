"""
pwlplan_test.jl
Description:
    Testing some of the algorithms developed in the STLPlanning repository.
"""

using Test

include("../src/PWLPlan.jl")

@testset "1. clearSpecTree" begin
    # Create a few sample nodes
    simpleNode1 = ConjunctionNode([1.0+2,0.1],[1.0],[2.0],"Simple Node 1")

    @test simpleNode1.Zs != []
    
    clearSpecTree(simpleNode1)
    @test simpleNode1.Zs == []

    # Create a more complicated example

end