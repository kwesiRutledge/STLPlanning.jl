"""
pwlplan_test.jl
Description:
    Testing some of the algorithms developed in the STLPlanning repository.
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

@testset "3. always Tests" begin
    # Create the inputs for always function
    i = 2

    a = 0.5
    b = 2.5

    pwl_curve = Vector{Vector{Float64}}(
        [[0.0,1.0,2.0],
         [1.0,2.0,3.0],
         [2.0,3.0,4.0],
         [3.0,4.0,5.0],
         [4.0,5.0,6.0]]
    )

    curve_length = length(pwl_curve)

    z_phis = Vector{Float64}([1.0,1,1,1,1])

    # Call always
    alwaysNode1 = always(i,a,b,z_phis,pwl_curve)
    
    # Test output node
    t_i = pwl_curve[i][1]
    t_i_1 = pwl_curve[i+1][1]
    for piece_index in range(1,stop=curve_length-1)
        t_j = pwl_curve[piece_index][1]
        t_j_1 = pwl_curve[piece_index+1][1]
        @test alwaysNode1.Children[piece_index].Children[1].Children[1] < t_i + a - t_j_1
    end

end

@testset "4. clearSpecTree" begin
    # Create a few sample nodes
    simpleNode1 = ConjunctionNode([1.0+2,0.1],[1.0],[2.0],"Simple Node 1")

    @test simpleNode1.Zs != []
    
    clearSpecTree(simpleNode1)
    @test simpleNode1.Zs == []

    # Create a more complicated example

end