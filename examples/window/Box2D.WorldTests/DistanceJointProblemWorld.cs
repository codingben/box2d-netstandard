using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.Joints.Distance;
using Box2D.NetStandard.Dynamics.World;

namespace Box2D.WorldTests
{
    [ExcludeFromCodeCoverage]
    public class DistanceJointProblemWorld
    {
        public static World CreateWorld()
        {
            // b2Vec2 g(0.000000000000000e+00f, -1.000000000000000e+01f);
            // m_world->SetGravity(g);

            var world = new World();
            var bodies = new Body[5];
            var joints = new Joint[1];

            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(-3.580528497695923e-02f, -7.826317548751831e-01f);
                bd.angle = 8.054048418998718e-01f;
                bd.linearVelocity = new Vector2(-2.581855468451977e-02f, 0.000000000000000e+00f);
                bd.angularVelocity = 5.164637044072151e-02f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.fixedRotation = false;
                bd.bullet = true;
                //bd.active = bool(32);
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[3] = world.CreateBody(bd);

                {
                    FixtureDef fd = new FixtureDef();
                    fd.friction = 2.000000029802322e-01f;
                    fd.restitution = 2.000000029802322e-01f;
                    fd.density = 2.000000000000000e+00f;
                    fd.isSensor = false;
                    fd.filter.categoryBits = (1);
                    fd.filter.maskBits = (65535);
                    fd.filter.groupIndex = (0);
                    CircleShape shape = new CircleShape();
                    shape.Radius = 5.000000000000000e-01f;
                    shape.Center = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);

                    fd.shape = shape;

                    bodies[3].CreateFixture(fd);
                }
            }
            {
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Static;
                bd.position = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angle = 0.000000000000000e+00f;
                bd.linearVelocity = new Vector2(0.000000000000000e+00f, 0.000000000000000e+00f);
                bd.angularVelocity = 0.000000000000000e+00f;
                bd.linearDamping = 0.000000000000000e+00f;
                bd.angularDamping = 0.000000000000000e+00f;
                bd.allowSleep = true;
                bd.fixedRotation = false;
                bd.bullet = false;
                //bd.active = bool(32);
                bd.gravityScale = 1.000000000000000e+00f;
                bodies[4] = world.CreateBody(bd);

            }
            {
                DistanceJointDef jd = new DistanceJointDef();
                jd.bodyA = bodies[4];
                jd.bodyB = bodies[3];
                jd.collideConnected = false;
                jd.localAnchorA = new Vector2(4.595311164855957e+00f, -1.409123420715332e+00f);
                jd.localAnchorB = new Vector2(4.783708136528730e-03f, 9.254086180590093e-04f);
                jd.length = 4.671227455139160e+00f;
                jd.frequencyHz = 0.000000000000000e+00f;
                jd.dampingRatio = 0.000000000000000e+00f;
                joints[0] = world.CreateJoint(jd);
            }

            return world;
        }
    }
}