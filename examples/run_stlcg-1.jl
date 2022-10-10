"""
run_stlcg-1.jl
Description:
    An implementation of the example in run_stlcg-1.py
    from the original STLPlanning repo.
"""

include("../src/PWLPlan.jl")

x0 = [  -1. ;
        1. ]

wall_half_width = 0.05
A = [   -1.0 0 ;
        1  0 ;
        0  -1;
        0  1 ]

walls = []
push!( walls , [-1.5; -1.5; -1.5; 1.5] )
push!( walls , [1.5; 1.5; -1.5; 1.5] )
push!( walls , [-1.5; 1.5; -1.5; -1.5] )
push!( walls , [-1.5; 1.5; 1.5; 1.5] )

obs = []
for wall in walls 
    if (wall[1] == wall[2])
        wall[1] -= wall_half_width
        wall[2] += wall_half_width
    elseif wall[3] == wall[4]
        wall[3] -= wall_half_width
        wall[4] += wall_half_width
    else
        throw(ErrorException("Wrong shape for axis-aligned wall"))
    end
    
    wall = wall .* [-1;1;-1;1]
    push!(obs,tuple(A,wall))
end

b1 = [1; -0.7; 0.25; 0.5]
B1 = tuple(A,b1)

b2 = [0; 0.9; 1; -0.5]
B2 = tuple(A,b2)

b3 = [-0.2; 0.7; -0.8; 1.2]
B3 = tuple(A,b3)

c1 = [0.4; 0.4; 0.4; 0.4]
C1 = tuple(A,c1)

# Goal
goal = [1;1]

# Create constraints
tmax = 15.0
vmax = 1.0

# Create specificationTree
notC = BasicNode("negmu", NodeInfo(C1[1],C1[2]))
B1_node = BasicNode("mu", NodeInfo(B1[1],B1[2]))
B2_node = BasicNode("mu", NodeInfo(B2[1],B2[2]))
B3_node = BasicNode("mu", NodeInfo(B3[1],B3[2]))

phi_1 = BasicNode("F", [BasicNode("A", [B2_node], NodeInfo(0.0,5.0))], NodeInfo(0.0,tmax) )
phi_2 = BasicNode("F", [BasicNode("A", [B3_node], NodeInfo(0.0,5.0))], NodeInfo(0.0,tmax) )
phi_3 = BasicNode("A", [notC], NodeInfo(0.0,tmax))
spec = BasicNode("and", [phi_1,phi_2,phi_3], NodeInfo())

# Call plan
x0s = [x0]
specs = [spec]
goals = [goal]
pwl_curves = plan(x0s, specs, 0.05 , [], 9, [], vmax, 1e-4, -1, tmax, [], 0.11*4/2)