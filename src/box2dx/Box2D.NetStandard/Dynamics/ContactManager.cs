/*
  Box2D.NetStandard Copyright © 2020 Ben Ukhanov & Hugh Phoenix-Hulme https://github.com/benzuk/box2d-netstandard
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  
// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
*/


using Box2D.NetStandard.Collision;
using Box2D.NetStandard.Dynamics.Body;
using Box2D.NetStandard.Dynamics.Contacts;
using Box2D.NetStandard.Dynamics.Fixture;
using int32 = System.Int32;
using b2Fixture = Box2D.NetStandard.Dynamics.Fixture.Fixture;
using b2Body = Box2D.NetStandard.Dynamics.Body.Body;
using b2Contact = Box2D.NetStandard.Dynamics.Contacts.Contact;

namespace Box2D.NetStandard.Dynamics {
  /// <summary>
  /// Delegate of World.
  /// </summary>
  internal class ContactManager {
    internal BroadPhase      m_broadPhase;
    internal b2Contact         m_contactList;
    internal int             m_contactCount;
    internal ContactFilter   m_contactFilter;
    internal ContactListener m_contactListener;

    internal ContactManager() {
      m_contactList     = null;
      m_contactCount    = 0;
      m_contactFilter   = new ContactFilter();
      m_contactListener = null;
      m_broadPhase      = new BroadPhase();
    }

    internal void Destroy(b2Contact c) {
      b2Fixture fixtureA = c.FixtureA;
      b2Fixture fixtureB = c.FixtureB;
      b2Body    bodyA    = fixtureA.Body;
      b2Body    bodyB    = fixtureB.Body;

      if (m_contactListener != null && c.Touching) {
        m_contactListener.EndContact(c);
      }

      // Remove from the world.
      if (c.m_prev != null) {
        c.m_prev.m_next = c.m_next;
      }

      if (c.m_next != null) {
        c.m_next.m_prev = c.m_prev;
      }

      if (c == m_contactList) {
        m_contactList = c.m_next;
      }

      // Remove from body 1
      if (c.m_nodeA.prev != null) {
        c.m_nodeA.prev.next = c.m_nodeA.next;
      }

      if (c.m_nodeA.next != null) {
        c.m_nodeA.next.prev = c.m_nodeA.prev;
      }

      if (c.m_nodeA == bodyA._contactList) {
        bodyA._contactList = c.m_nodeA.next;
      }

      // Remove from body 2
      if (c.m_nodeB.prev != null) {
        c.m_nodeB.prev.next = c.m_nodeB.next;
      }

      if (c.m_nodeB.next != null) {
        c.m_nodeB.next.prev = c.m_nodeB.prev;
      }

      if (c.m_nodeB == bodyB._contactList) {
        bodyB._contactList = c.m_nodeB.next;
      }

      // Call the factory.
      Contact.Destroy(ref c);
      --m_contactCount;
    }

    internal void Collide() {
      // Update awake contacts.
      b2Contact c = m_contactList;
      while (c != null) {
        b2Fixture fixtureA = c.FixtureA;
        b2Fixture fixtureB = c.FixtureB;
        int32     indexA   = c.ChildIndexA;
        int32     indexB   = c.ChildIndexB;
        b2Body    bodyA    = fixtureA.Body;
        b2Body    bodyB    = fixtureB.Body;

        // Is this contact flagged for filtering?
        if ((c.m_flags & CollisionFlags.Filter) == CollisionFlags.Filter) {
          // Should these bodies collide?
          if (bodyB.ShouldCollide(bodyA) == false) {
            b2Contact cNuke = c;
            c = cNuke.GetNext();
            Destroy(cNuke);
            continue;
          }

          // Check user filtering.
          if (m_contactFilter != null && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
            b2Contact cNuke = c;
            c = cNuke.GetNext();
            Destroy(cNuke);
            continue;
          }

          // Clear the filtering flag.
          c.m_flags &= ~CollisionFlags.Filter;
        }

        bool activeA = bodyA.IsAwake() && bodyA._type != BodyType.Static;
        bool activeB = bodyB.IsAwake() && bodyB._type != BodyType.Static;

        // At least one body must be awake and it must be dynamic or kinematic.
        if (activeA == false && activeB == false) {
          c = c.GetNext();
          continue;
        }

        int32 proxyIdA = fixtureA.m_proxies[indexA].proxyId;
        int32 proxyIdB = fixtureB.m_proxies[indexB].proxyId;
        bool  overlap  = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

        // Here we destroy contacts that cease to overlap in the broad-phase.
        if (overlap == false) {
          b2Contact cNuke = c;
          c = cNuke.GetNext();
          Destroy(cNuke);
          continue;
        }

        // The contact persists.
        c.Update(m_contactListener);
        c = c.GetNext();
      }
    }

    internal void FindNewContacts() {
      m_broadPhase.UpdatePairs(AddPair);
    }

    void AddPair(object proxyUserDataA, object proxyUserDataB) {
      FixtureProxy proxyA = (FixtureProxy) proxyUserDataA;
      FixtureProxy proxyB = (FixtureProxy) proxyUserDataB;

      b2Fixture fixtureA = proxyA.fixture;
      b2Fixture fixtureB = proxyB.fixture;

      int32 indexA = proxyA.childIndex;
      int32 indexB = proxyB.childIndex;

      b2Body bodyA = fixtureA.Body;
      b2Body bodyB = fixtureB.Body;

      // Are the fixtures on the same body?
      if (bodyA == bodyB) {
        return;
      }

      // TODO_ERIN use a hash table to remove a potential bottleneck when both
      // bodies have a lot of contacts.
      // Does a contact already exist?
      ContactEdge edge = bodyB.GetContactList();
      while (edge != null) {
        if (edge.other == bodyA) {
          b2Fixture fA = edge.contact.GetFixtureA();
          b2Fixture fB = edge.contact.GetFixtureB();
          int32     iA = edge.contact.GetChildIndexA();
          int32     iB = edge.contact.GetChildIndexB();

          if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB) {
            // A contact already exists.
            return;
          }

          if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA) {
            // A contact already exists.
            return;
          }
        }

        edge = edge.next;
      }

      // Does a joint override collision? Is at least one body dynamic?
      if (bodyB.ShouldCollide(bodyA) == false) {
        return;
      }

      // Check user filtering.
      if (m_contactFilter != null && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
        return;
      }

      // Call the factory.
      Contact c = Contact.Create(fixtureA, indexA, fixtureB, indexB);
      if (c == null) {
        return;
      }

      // Contact creation may swap fixtures.
      fixtureA = c.GetFixtureA();
      fixtureB = c.GetFixtureB();
      indexA   = c.GetChildIndexA();
      indexB   = c.GetChildIndexB();
      bodyA    = fixtureA.GetBody();
      bodyB    = fixtureB.GetBody();

      // Insert into the world.
      c.m_prev = null;
      c.m_next = m_contactList;
      if (m_contactList != null) {
        m_contactList.m_prev = c;
      }

      m_contactList = c;

      // Connect to island graph.

      // Connect to body A
      c.m_nodeA.contact = c;
      c.m_nodeA.other   = bodyB;

      c.m_nodeA.prev = null;
      c.m_nodeA.next = bodyA._contactList;
      if (bodyA._contactList != null) {
        bodyA._contactList.prev = c.m_nodeA;
      }

      bodyA._contactList = c.m_nodeA;

      // Connect to body B
      c.m_nodeB.contact = c;
      c.m_nodeB.other   = bodyA;

      c.m_nodeB.prev = null;
      c.m_nodeB.next = bodyB._contactList;
      if (bodyB._contactList != null) {
        bodyB._contactList.prev = c.m_nodeB;
      }

      bodyB._contactList = c.m_nodeB;

      ++m_contactCount;
    }
  }
}