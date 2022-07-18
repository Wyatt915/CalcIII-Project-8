from manim import *

class LineIntegral(Scene):
    def construct(self):
        force_field = lambda pos: (pos[0] * pos[0] * RIGHT) - (pos[0] * pos[1] * UP)
        point = Dot()
        # vf = ArrowVectorField(force_field)
        # self.add(vf)
        circ = Circle(radius=2)
        stream_lines = StreamLines(force_field, stroke_width=2, max_anchors_per_line=90,
                max_color_scheme_value=np.linalg.norm(force_field([config.frame_width/2,config.frame_height/2])))
        self.add(stream_lines)
        stream_lines.start_animation(warm_up=False, flow_speed=1.5)
        self.play(Create(circ))
        self.vel = np.array([0.0, 10.0, 0.0])
        position = np.array([2.0, 0.0, 0])
        pm = Dot(position)
        pm.add_updater(lambda mob, dt: self.verlet(mob, dt, force_field, substeps=10))
        self.add(pm)
        #self.play(MoveAlongPath(pm, circ), rate_func=lambda x: x/5)
        self.wait(20)

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
        pos = mob.get_center()
        for i in range(substeps):
            pos += dt * self.vel
            # constrain position to the circle
            pos = radius * pos / np.linalg.norm(pos)
            self.vel += dt * force_field(pos)
            # constrain velocity to be tangent to the circle
            tangent = np.array([-1 * pos[1], pos[0], 0.0])
            tangent /= np.linalg.norm(tangent) #normalized to magnitude 1
            self.vel = np.dot(self.vel, tangent) * tangent
            self.vel *= 0.999 # drag
        mob.set_x(pos[0])
        mob.set_y(pos[1])
        return mob

    def riemann(self, curve, steps):
        pass

    def delta_W(self, init_pos, final_pos, force_field):
        delta_r = final_pos - init_pos
        force_vec = force_field(final_pos)
        return np.dot(delta_r, force_vec)
