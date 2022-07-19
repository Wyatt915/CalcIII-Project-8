from manim import *

FORCE_FIELDS = {
        "assigned": lambda pos: (pos[0] * pos[0] * RIGHT) + (pos[0] * pos[1] * UP),
        "textbook": lambda pos: (pos[0] * pos[0] * RIGHT) - (pos[0] * pos[1] * UP)
        }

FIELD_EQNS = {
        "assigned": MathTex(r"\mathbf{F}(x,y)=x^2\hat{\imath} + xy\hat{\jmath}"),
        "textbook": MathTex(r"\mathbf{F}(x,y)=x^2\hat{\imath} - xy\hat{\jmath}")
        }

CIRCLE_EQNS = {
        "implicit": MathTex(r"x^2 + y^2 = 2^2"),
        "parametric": MathTex(r"\mathbf{r}(t) = \langle 2\cos{t}, 2\sin{t} \rangle")
        }

RADIUS = 2.0

class PointMass(Dot):
    def set_vel(self, velocity):
        self.vel = np.array(velocity)
    def set_pos(self, position):
        self.pos = np.array([position[0], position[1], 0.0])
        self.set_x(position[0])
        self.set_y(position[1])
        self.set_z(0)
    def get_vel(self):
        return self.vel
    def get_pos(self):
        return self.pos

class LineIntegral(Scene):
    def construct(self):
        force_field = FORCE_FIELDS["textbook"]
        #field_desc = FIELD_EQNS["textbook"]

        ## create vector field
        #vf = ArrowVectorField(force_field)
        #self.play(Create(vf), Write(field_desc))
        #field_desc.generate_target()
        #field_desc.target.to_edge(UP)
        #self.play(MoveToTarget(field_desc))


        ## replace vector field with stream lines
        #stream_lines = StreamLines(force_field, stroke_width=2, max_anchors_per_line=30,
        #        max_color_scheme_value=np.linalg.norm(force_field([config.frame_width/2,config.frame_height/2])))
        #stream_lines.set_z_index(-1)
        #self.add(stream_lines)
        #stream_lines.start_animation(warm_up=False, flow_speed=1.5)
        #self.play(Uncreate(vf))

        # create circle and point
        circ = Circle(radius=RADIUS)
        point = PointMass()
        CIRCLE_EQNS["implicit"].next_to(circ, RIGHT)
        CIRCLE_EQNS["parametric"].next_to(circ, RIGHT)
        self.play(Create(circ), Write(CIRCLE_EQNS["implicit"]))
        self.wait(2)
        self.play(Transform(CIRCLE_EQNS["implicit"], CIRCLE_EQNS["parametric"]))

        point.set_vel([0.0, 10.0, 0.0])
        point.set_pos([RADIUS, 0.0, 0])
        #point.add_updater(lambda mob, dt: self.verlet(mob, dt, force_field, substeps=10))
        self.add(point)

        steps = 20
        time_around_circle = 3 # seconds
        path_length = 2 * PI
        theta = path_length / steps
        point.add_updater(lambda x: x.set_pos(x.get_center()))
        gradient = color_gradient([GREEN, BLUE], steps)
        for i in range(steps):
            progress = i / steps
            begin = point.get_pos()
            self.play(MoveAlongPath(point, circ),
                    run_time=time_around_circle/steps,
                    rate_func=lambda x:progress + x/steps)
            work, rect = self.line_integral_partition(
                    begin, point.get_pos(),
                    progress * path_length, (progress*path_length) + theta,
                    gradient[i],
                    force_field
            )
            #point.set_pos(new_pos)
            self.play(Create(rect))

    def verlet(self, mob, dt, force_field, substeps = 1, radius=2):
        """
        Verlet integrator - Calculate force on the point and update its
        position and velocity accordingly.

        mob: Mobject - the point moving around the circle
        dt: Time since this function was last called
        force_field: Vector force field experienced by the object
        substeps: Higher value increases the accuracy of the integration
        radius: Radius of the constraining circle
        """
        dt /= substeps
        pos = mob.get_pos()
        vel = mob.get_vel()
        for i in range(substeps):
            pos += dt * vel
            # constrain position to the circle
            pos = radius * pos / np.linalg.norm(pos)
            vel += dt * force_field(pos)
            # constrain velocity to be tangent to the circle
            # `tangent` is a unit vector giving the direction of velocity tangent to the circle.
            tangent = np.array([-1 * pos[1], pos[0], 0.0])
            tangent /= np.linalg.norm(tangent) #normalized to magnitude 1
            vel = np.dot(vel, tangent) * tangent
            vel *= 0.999 # drag
        mob.set_pos(pos)
        mob.set_vel(vel)
        return mob

    def line_integral_partition(self, begin, end, t_beg, t_end, rect_color, force_field):
        """
        Calculate the rectangle of area for the line integral, given some Δr vector.

        begin (np.array): the initial position on the path
        end (np.array): the final position on the path
        force_field: force field doing work on the point
        t_beg: initial time
        t_end: final time
        t_total: total "length" of the path
        progress: float on range [0,1] representing how far along the path we
                  are (used to calculate color for the rectangle)

        returns: work done over this interval, rectangle representing area under curve
        """
        delta = end - begin
        work = np.dot(force_field(end), delta)
        height = work / np.linalg.norm(end - begin)
        vertex_list = [
                [t_beg, 0,      0],  # left x-axis
                [t_end, 0,      0],  # right x-axis
                [t_end, height, 0],  # right top-axis
                [t_beg, height, 0]   # left top
        ]
        rect = Polygon(*vertex_list).set_fill(rect_color, 0.9).set_stroke(width=0)
        return work, rect

    def full_rotation(self, mob, radius, steps, force_field):
        radians = 2 * PI / steps
        for i in range(steps):
            work, old_pos, new_pos = move_around_circle(mob, radius, radians, force_field)


