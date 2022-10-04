"""
pwlplan_test.jl
Description:
    Testing some of the algorithms developed in the STLPlanning repository.
"""

using Test

include("../src/PWLPlan.jl")

@testset "1. always Tests" begin
    # Create the inputs for always function
    i = 2

    a = 0.5
    b = 2.5

    pwl_curve = Vector{Tuple{Vector{Float64},Float64}}(
        [([0.0,1.0,2.0],0.0),
         ([1.0,2.0,3.0],1.0),
         ([2.0,3.0,4.0],2.0),
         ([3.0,4.0,5.0],3.0),
         ([4.0,5.0,6.0],4.0)]
    )

    curve_length = length(pwl_curve)

    z_phis = Vector{Float64}([1.0,1,1,1,1])

    # Call always
    alwaysNode1 = always(i,a,b,z_phis,pwl_curve)
    
    # Test output node
    t_i = pwl_curve[i][2]
    t_i_1 = pwl_curve[i+1][2]
    for piece_index in range(1,stop=curve_length-1)
        t_j = pwl_curve[piece_index][2]
        t_j_1 = pwl_curve[piece_index+1][2]
        @test alwaysNode1.Children[piece_index].Children[1].Children[1] < t_i + a - t_j_1
    end

end

@testset "2. clearSpecTree" begin
    # Create a few sample nodes
    simpleNode1 = ConjunctionNode([1.0+2,0.1],[1.0],[2.0],"Simple Node 1")

    @test simpleNode1.Zs != []
    
    clearSpecTree(simpleNode1)
    @test simpleNode1.Zs == []

    # Create a more complicated example

end

@testset "3. bounded_eventually testing" begin
    # Create two symbolic variables
    pwl_curve = Vector{Tuple{Vector{Any},Any}}(
        [([0.0,1.0,2.0],0.0),
         ([1.0,2.0,3.0],1.0),
         ([2.0,3.0,4.0],2.0),
         ([3.0,4.0,5.0],3.0),
         ([4.0,5.0,6.0],4.0)]
    )

    a = 0.5
    b = 2.5

    tmax = 3.7

    node1 = ConjunctionNode([1.0+2,0.1],[1.0],[2.0],"Simple Node 1")
    node2 = DisjunctionNode([5.0,-9.1],[],[],"Simple Node 2")

    # Algorithm 
    t_i = 0.0
    be_node = bounded_eventually(2,a,b,Vector{Float64}([1.0,2.0,3.0,4.0,5.0]),pwl_curve,tmax)
    @test be_node.Children[2] > t_i+b-tmax
end