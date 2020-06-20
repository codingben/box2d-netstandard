namespace Box2D.NetStandard.Dynamics.Contacts {
  /// <summary>
  /// A contact edge is used to connect bodies and contacts together
  /// in a contact graph where each body is a node and each contact
  /// is an edge. A contact edge belongs to a doubly linked list
  /// maintained in each attached body. Each contact has two contact
  /// nodes, one for each attached body.
  /// </summary>
  public class ContactEdge {
    /// <summary>
    /// Provides quick access to the other body attached.
    /// </summary>
    internal Body.Body other;

    /// <summary>
    /// The contact.
    /// </summary>
    internal Contact contact;

    /// <summary>
    /// The previous contact edge in the body's contact list.
    /// </summary>
    internal ContactEdge prev;

    /// <summary>
    /// The next contact edge in the body's contact list.
    /// </summary>
    internal ContactEdge next;
  }
}