from manim import *

class ContinuousMotion(Scene):
    def construct(self):
        force_field = lambda pos: (pos[0] ** 2) * RIGHT + (pos[0] * pos[1]) * UP
        d_force_field = lambda pos: (2 * pos[0]) * RIGHT + (pos[0]) * UP
        point = Dot()
        #vf = ArrowVectorField(force_field)
        #self.add(vf)
        stream_lines = StreamLines(force_field, stroke_width=2, max_anchors_per_line=90,
                max_color_scheme_value=np.linalg.norm(force_field([config.frame_width/2,config.frame_height/2])))
        self.add(stream_lines)
        stream_lines.start_animation(warm_up=False, flow_speed=1.5)
        self.wait(stream_lines.virtual_time / stream_lines.flow_speed)
        self.vel = np.array([-10.0,0.0,0.0])
        pm = Dot([2.,2.,0.])
        pm.add_updater(lambda mob, dt: self.euler(mob, dt, force_field, substeps=1))
        self.add(pm)
        self.wait(5)

    def euler(self, mob, dt, force_field, substeps = 1):
        timestep = dt
        timestep /= substeps
        pos = mob.get_center()
        for i in range(substeps):
            dt += timestep
            pos += dt * self.vel
            self.vel += dt * force_field(pos)
        mob.set_x(pos[0])
        mob.set_y(pos[1])
        return mob
