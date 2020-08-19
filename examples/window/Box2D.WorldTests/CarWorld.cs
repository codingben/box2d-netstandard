using System;
using System.Numerics;
using Box2D.NetStandard.Collision.Shapes;
using Box2D.NetStandard.Dynamics.Bodies;
using Box2D.NetStandard.Dynamics.Fixtures;
using Box2D.NetStandard.Dynamics.Joints;
using Box2D.NetStandard.Dynamics.Joints.Revolute;
using Box2D.NetStandard.Dynamics.Joints.Wheel;
using Box2D.NetStandard.Dynamics.World;
using b2Vec2 = System.Numerics.Vector2;
using int32 = System.Int32;

namespace Box2D.WorldTests
{
    public static class CarWorld
    {
        public static World CreateWorld(out Body[] bodies, out Joint[] joints)
        {
            // m_speed = 50.0f;
            bodies = new Body[9];
            joints = new Joint[3];

            var world = new World();

            Body ground;

            {
                BodyDef bd = new BodyDef();
                ground = world.CreateBody(bd);

                EdgeShape shape = new EdgeShape();

                FixtureDef fd = new FixtureDef();
                fd.shape = shape;
                fd.density = 0.0f;
                fd.friction = 0.6f;

                shape.SetTwoSided(new b2Vec2(-20.0f, 0.0f), new b2Vec2(20.0f, 0.0f));
                ground.CreateFixture(fd);

                float[] hs = { 0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f };

                float x = 20.0f, y1 = 0.0f, dx = 5.0f;

                for (int32 i = 0; i < 10; ++i)
                {
                    float y2 = hs[i];

                    shape.SetTwoSided(new b2Vec2(x, y1), new b2Vec2(x + dx, y2));
                    ground.CreateFixture(fd);

                    y1 = y2;
                    x += dx;
                }

                for (int32 i = 0; i < 10; ++i)
                {
                    float y2 = hs[i];

                    shape.SetTwoSided(new b2Vec2(x, y1), new b2Vec2(x + dx, y2));
                    ground.CreateFixture(fd);

                    y1 = y2;
                    x += dx;
                }

                shape.SetTwoSided(new b2Vec2(x, 0.0f), new b2Vec2(x + 40.0f, 0.0f));
                ground.CreateFixture(fd);

                x += 80.0f;
                shape.SetTwoSided(new b2Vec2(x, 0.0f), new b2Vec2(x + 40.0f, 0.0f));
                ground.CreateFixture(fd);

                x += 40.0f;
                shape.SetTwoSided(new b2Vec2(x, 0.0f), new b2Vec2(x + 10.0f, 5.0f));
                ground.CreateFixture(fd);

                x += 20.0f;
                shape.SetTwoSided(new b2Vec2(x, 0.0f), new b2Vec2(x + 40.0f, 0.0f));
                ground.CreateFixture(fd);

                x += 40.0f;
                shape.SetTwoSided(new b2Vec2(x, 0.0f), new b2Vec2(x, 20.0f));
                ground.CreateFixture(fd);
            }

            // Teeter
            {
                BodyDef bd = new BodyDef();
                bd.position = new b2Vec2(140.0f, 1.0f);
                bd.type = BodyType.Dynamic;
                Body body = world.CreateBody(bd);
                bodies[0] = body;

                PolygonShape box = new PolygonShape();
                box.SetAsBox(10.0f, 0.25f);
                body.CreateFixture(box, 1.0f);

                RevoluteJointDef jd = new RevoluteJointDef();
                jd.Initialize(ground, body, body.GetPosition());
                jd.lowerAngle = -8.0f * MathF.PI / 180.0f;
                jd.upperAngle = 8.0f * MathF.PI / 180.0f;
                jd.enableLimit = true;
                Joint j = world.CreateJoint(jd);
                joints[2] = j;
                body.ApplyAngularImpulse(100.0f, true);
            }

            // Bridge
            {
                int32 N = 20;
                PolygonShape shape = new PolygonShape();
                shape.SetAsBox(1.0f, 0.125f);

                FixtureDef fd = new FixtureDef();
                fd.shape = shape;
                fd.density = 1.0f;
                fd.friction = 0.6f;

                RevoluteJointDef jd = new RevoluteJointDef();

                Body prevBody = ground;
                for (int32 i = 0; i < N; ++i)
                {
                    BodyDef bd = new BodyDef();
                    bd.type = BodyType.Dynamic;
                    bd.position = new Vector2(161.0f + 2.0f * i, -0.125f);
                    Body body = world.CreateBody(bd);
                    body.CreateFixture(fd);

                    b2Vec2 anchor = new b2Vec2(160.0f + 2.0f * i, -0.125f);
                    jd.Initialize(prevBody, body, anchor);
                    world.CreateJoint(jd);

                    prevBody = body;
                }

                {
                    b2Vec2 anchor = new b2Vec2(160.0f + 2.0f * N, -0.125f);
                    jd.Initialize(prevBody, ground, anchor);
                    world.CreateJoint(jd);
                }
            }

            // Boxes
            {
                PolygonShape box = new PolygonShape();
                box.SetAsBox(0.5f, 0.5f);

                Body body = null;
                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;

                bd.position = new Vector2(230.0f, 0.5f);
                body = world.CreateBody(bd);
                bodies[1] = body;
                body.CreateFixture(box, 0.5f);

                bd.position = new Vector2(230.0f, 1.5f);
                body = world.CreateBody(bd);
                bodies[2] = body;
                body.CreateFixture(box, 0.5f);

                bd.position = new Vector2(230.0f, 2.5f);
                body = world.CreateBody(bd);
                bodies[3] = body;
                body.CreateFixture(box, 0.5f);

                bd.position = new Vector2(230.0f, 3.5f);
                body = world.CreateBody(bd);
                bodies[4] = body;
                body.CreateFixture(box, 0.5f);

                bd.position = new Vector2(230.0f, 4.5f);
                body = world.CreateBody(bd);
                bodies[5] = body;
                body.CreateFixture(box, 0.5f);
            }

            // Car
            Body m_car;
            Body m_wheel1;
            Body m_wheel2;
            WheelJoint m_spring1;
            WheelJoint m_spring2;

            {
                PolygonShape chassis = new PolygonShape();
                b2Vec2[] vertices = new b2Vec2[6];
                vertices[0] = new Vector2(-1.5f, -0.5f);
                vertices[1] = new Vector2(1.5f, -0.5f);
                vertices[2] = new Vector2(1.5f, 0.0f);
                vertices[3] = new Vector2(0.0f, 0.9f);
                vertices[4] = new Vector2(-1.15f, 0.9f);
                vertices[5] = new Vector2(-1.5f, 0.2f);
                chassis.Set(vertices);

                CircleShape circle = new CircleShape();
                circle.Radius = 0.4f;

                BodyDef bd = new BodyDef();
                bd.type = BodyType.Dynamic;
                bd.position = new Vector2(0.0f, 1.0f);

                m_car = world.CreateBody(bd);
                m_car.CreateFixture(chassis, 1.0f);

                FixtureDef fd = new FixtureDef();
                fd.shape = circle;
                fd.density = 1.0f;
                fd.friction = 0.9f;

                bd.position = new Vector2(-1.0f, 0.35f);
                m_wheel1 = world.CreateBody(bd);
                m_wheel1.CreateFixture(fd);

                bd.position = new Vector2(1.0f, 0.4f);
                m_wheel2 = world.CreateBody(bd);
                m_wheel2.CreateFixture(fd);

                WheelJointDef jd = new WheelJointDef();
                b2Vec2 axis = new Vector2(0.0f, 1.0f);

                float mass1 = m_wheel1.GetMass();
                float mass2 = m_wheel2.GetMass();

                float hertz = 4.0f;
                float dampingRatio = 0.7f;
                float omega = 2.0f * MathF.PI * hertz;

                jd.Initialize(m_car, m_wheel1, m_wheel1.GetPosition(), axis);
                jd.motorSpeed = 0.0f;
                jd.maxMotorTorque = 20.0f;
                jd.enableMotor = true;
                jd.stiffness = mass1 * omega * omega;
                jd.damping = 2.0f * mass1 * dampingRatio * omega;
                jd.lowerTranslation = -0.25f;
                jd.upperTranslation = 0.25f;
                jd.enableLimit = true;
                m_spring1 = (WheelJoint)world.CreateJoint(jd);

                jd.Initialize(m_car, m_wheel2, m_wheel2.GetPosition(), axis);
                jd.motorSpeed = 0.0f;
                jd.maxMotorTorque = 10.0f;
                jd.enableMotor = false;
                jd.stiffness = mass2 * omega * omega;
                jd.damping = 2.0f * mass2 * dampingRatio * omega;
                jd.lowerTranslation = -0.25f;
                jd.upperTranslation = 0.25f;
                jd.enableLimit = true;

                m_spring2 = (WheelJoint)world.CreateJoint(jd);
            }

            joints[0] = m_spring1;
            joints[1] = m_spring2;
            bodies[6] = m_wheel1;
            bodies[7] = m_wheel2;
            bodies[8] = m_car;
            m_spring1.SetMotorSpeed(-40);

            return world;
        }
    }
}