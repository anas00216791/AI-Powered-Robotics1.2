// In-memory storage for users (when MongoDB is not available)
class MemoryStore {
  constructor() {
    this.users = new Map();
    this.counter = 1;
  }

  async createUser(userData) {
    const id = String(this.counter++);
    const user = {
      _id: id,
      id: id,
      ...userData,
      createdAt: new Date(),
      updatedAt: new Date(),
      isVerified: false,
      role: 'student',
      enrolledModules: [],
      progress: {}
    };

    this.users.set(user.email, user);
    return user;
  }

  async findByEmail(email) {
    return this.users.get(email) || null;
  }

  async findById(id) {
    for (const user of this.users.values()) {
      if (user._id === id || user.id === id) {
        return user;
      }
    }
    return null;
  }

  async updateUser(email, updates) {
    const user = this.users.get(email);
    if (!user) return null;

    Object.assign(user, updates, { updatedAt: new Date() });
    this.users.set(email, user);
    return user;
  }

  async deleteUser(email) {
    return this.users.delete(email);
  }

  getAllUsers() {
    return Array.from(this.users.values());
  }

  clear() {
    this.users.clear();
    this.counter = 1;
  }
}

// Singleton instance
const memoryStore = new MemoryStore();

module.exports = memoryStore;
