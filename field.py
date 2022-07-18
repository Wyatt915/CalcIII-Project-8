from manim import *

FORCE_FIELDS = {
        "assigned": lambda pos: (pos[0] * pos[0] * RIGHT) + (pos[0] * pos[1] * UP),
        "textbook": lambda pos: (pos[0] * pos[0] * RIGHT) - (pos[0] * pos[1] * UP)
        }

FIELD_EQNS = {
        "assigned": r"\mathbf{F}(x,y)=x^2\hat{\imath} + xy\hat{\jmath}",
        "textbook": r"\mathbf{F}(x,y)=x^2\hat{\imath} - xy\hat{\jmath}"
        }

class PointMass(Dot):
    def set_vel(self, velocity):
        self.vel = np.array(velocity)
    def set_pos(self, position):
        self.pos = np.array(position)
        self.set_x(position[0])
        self.set_y(position[1])
    def get_vel(self):
        return self.vel
    def get_pos(self):
        return self.pos

class LineIntegral(Scene):
    def construct(self):
        point = PointMass()
        force_field = FORCE_FIELDS["textbook"]
        field_desc = MathTex(FIELD_EQNS["textbook"])
        vf = ArrowVectorField(force_field)
        self.play(Create(vf), Write(field_desc))
        self.play(field_desc.animate.shift(UP*3.5))



        circ = Circle(radius=2)
        stream_lines = StreamLines(force_field, stroke_width=2, max_anchors_per_line=10,
                max_color_scheme_value=np.linalg.norm(force_field([config.frame_width/2,config.frame_height/2])))
        stream_lines.set_z_index(-1)
        self.add(stream_lines)
        stream_lines.start_animation(warm_up=False, flow_speed=1.5)
        self.play(Uncreate(vf))
        self.play(Create(circ))
        point.set_vel([0.0, 10.0, 0.0])
        point.set_pos([2.0, 0.0, 0])
        #point.add_updater(lambda mob, dt: self.verlet(mob, dt, force_field, substeps=10))
        self.add(point)
        #self.play(MoveAlongPath(pm, circ), rate_func=lambda x: x/5)
        steps = 3
        time_around_circle = 3 # seconds
        radians = 2 * PI / steps
        for i in range(steps):
            #work, old_pos, new_pos = self.move_around_circle(point, 2, radians, force_field)
            progress = i/steps
            self.play(MoveAlongPath(point, circ),
                    run_time=time_around_circle/steps,
                    rate_func=lambda x:progress + x/steps)
            #point.set_pos(new_pos)
            self.wait(1)

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

    def move_around_circle(self, mob, radius, radians, force_field):
        """
        Move a Mobject around a sector of a circle, and compute the approximate
        work done on the object by a force field. Does not update the position of the Mobject.

        mob:     object to move around the circle
        radius:  radius of the circle
        radians: distance around the circle to move
        force_field: field doing work on the object
        """
        pos = mob.get_pos()
        theta = np.arccos(pos[0]/radius)
        if pos[1] < 0:
            theta += PI
        theta += radians
        new_pos = radius * np.array([np.cos(theta), np.sin(theta), 0])
        delta = new_pos - pos
        work = np.dot(force_field(new_pos), delta)
        # mob.set_pos(new_pos)
        return work, pos, new_pos

    def full_rotation(self, mob, radius, steps, force_field):
        radians = 2 * PI / steps
        for i in range(steps):
            work, old_pos, new_pos = move_around_circle(mob, radius, radians, force_field)


