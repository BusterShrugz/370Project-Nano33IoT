/*
 * This file is part of Arduino_KVStore.
 *
 * Copyright (c) 2024 Arduino SA
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif // ARDUINO

#include <math.h>
#include <type_traits>

/** KVStoreInterface class
 *
 * Interface for HW abstraction of a KV store
 *
 * In order to implement this interface it is just required to implement just the pure virtual methods
 * and as a consequence all the other methods will work out of the box.
 * If some datatypes have optimizations, put and get methods can be overwritten and specialized for them
 */
class KVStoreInterface {
public:

    typedef const char* Key;
    typedef Key key_t;
    typedef int res_t;

    typedef enum {
        PT_I8, PT_U8, PT_I16, PT_U16, PT_I32, PT_U32, PT_I64, PT_U64, PT_STR, PT_BLOB, PT_INVALID, PT_FLOAT, PT_DOUBLE,
    } Type;

    // TODO this is an utility function for kvstore should this stay here?
    /**
     * @brief This function translate a cpp kind to a Preferences Type at compile time
     *
     * @returns the corresponding Type of the passed parameter
     */
    template<typename T>
    static constexpr Type getType(T t);

    // proxy class to allow operator[] with assignment

    /** reference class
     *
     * This class is a container that holds a key-value pair and provides
     * access to it in both read and write mode
     */
    template<typename T>
    class reference {
    public:
        reference(const key_t &key, const T& value, KVStoreInterface& owner)
        : key(key), value(value), owner(owner) {}

        // assign a new value to the reference and update the store
        reference& operator=(T t) noexcept {
            value = t;
            this->save();
            return *this;
        }

        // assign a new value to the reference copying from another reference value
        reference& operator=(const reference<T>& r) noexcept {
            value = r.value;
            return *this;
        }

        // get the referenced value
        T operator*() const noexcept { return getValue(); }

        // cast the reference to the value it contains -> get the value references
        operator T () const noexcept { return getValue(); }

        inline key_t getKey() const  { return key; }
        inline T getValue() const    { return value; }

        // load the stored value
        void load()                  { value = owner.get<T>(key).value; }

        // save the value contained in this reference
        void save()                  { owner.put(key, value); }

        // check if this reference is contained in the store
        bool exists() const          { return owner.exists(key); }

        // remove this reference from the store
        res_t remove()                { return owner.remove(key); }
    private:
        const key_t key;
        T value;

        KVStoreInterface& owner;
    };

    /**
     * @brief virtual empty destructor
     */
    virtual ~KVStoreInterface() {};

    /**
     * @brief function that provides initializatiopn for the KV store
     *
     * @returns true on correct execution false otherwise
     */
    virtual bool begin() = 0;

    /**
     * @brief function that provides finalization for the KV store
     *
     * @returns true on correct execution false otherwise
     */
    virtual bool end() = 0;

    /**
     * @brief function that clears all the content of the KV store
     *
     * @returns true on correct execution false otherwise
     */
    virtual bool clear() = 0;

    /**
     * @brief remove what is referenced with the key
     *
     * @param[in]  key              Key which data has to be deleted
     *
     * @returns 1 on correct execution anything else otherwise
     */
    virtual res_t remove(const key_t& key) = 0;

    /**
     * @brief check if a key is already being used
     *
     * @param[in]  key              Key to search for
     *
     * @returns 1 on correct execution anything else otherwise
     */
    virtual bool exists(const key_t& key) const = 0;

    /**
     * @brief put values in the store provinding a byte array
     *
     * @param[in]  key              Key to insert
     * @param[in]  b                byte array
     * @param[in]  s                the length of the array
     *
     * @returns 1 on correct execution anything else otherwise
     */
    virtual res_t putBytes(const key_t& key, const uint8_t b[], size_t s) = 0;

    /**
     * @brief get values from the store provinding a byte array
     *
     * @param[in]  key              Key to get
     * @param[out] b                byte array
     * @param[in]  s                the length of the array
     *
     * @returns 1 on correct execution anything else otherwise
     */
    virtual res_t getBytes(const key_t& key, uint8_t b[], size_t s) const = 0;

    /**
     * @brief get the number of bytes used by a certain value referenced by key
     *
     * @param[in]  key              Key
     *
     * @returns len>0 if the key exists 0 otherwise
     */
    virtual size_t getBytesLength(const key_t& key) const = 0;

    /**
     * @brief templated method that puts a value of a certain type T
     *        by converting it to a bytearray and puts it into the KV store
     *
     * @param[in]  key              Key
     * @param[in]  value            Value to insert
     *
     * @returns 1 on correct execution anything else otherwise
     */
    template<typename T> // TODO handle std::string
    res_t put(const key_t& key, T value);

    /**
     * @brief templated method that gets a value of a certain type T. If it doesn't exist in the store a
     *        reference is returned, which is not saved, until the proper method is called
     *
     * @param[in]  key              Key
     * @param[in]  def              a default value that is assigned to the reference object
     *
     * @returns a reference to the desired key
     */
    template<typename T> // TODO this could be called when class is const
    reference<T> get(const key_t& key, const T def = 0);
    /**
     * @brief RW direct access to a value with the operator[]
     *
     * @param[in]  key              Key
     *
     * @returns a reference to the desired key
     */
    template<typename T>
    inline reference<T>& operator[](const key_t& key) { // write access to the value
        return get<T>(key);
    }

    /**
     * @brief R direct access to a value with the operator[]
     *
     * @param[in]  key              Key
     *
     * @returns a read-only reference to the desired key
     */
    template<typename T>
    inline const reference<T>& operator[](const key_t& key) const { // ro access to the value
        return get<T>(key);
    }

    // TODO all these methods should be const
    /**
     * @brief put a char in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putChar(const key_t& key, const int8_t value);

    /**
     * @brief put a uchar in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putUChar(const key_t& key, const uint8_t value);

    /**
     * @brief put a short in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putShort(const key_t& key, const int16_t value);

    /**
     * @brief put a ushort in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putUShort(const key_t& key, const uint16_t value);

    /**
     * @brief put an int in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putInt(const key_t& key, const int32_t value);

    /**
     * @brief put a uint in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putUInt(const key_t& key, const uint32_t value);

    /**
     * @brief put a long in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putLong(const key_t& key, const int32_t value);

    /**
     * @brief put a ulong in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putULong(const key_t& key, const uint32_t value);

    /**
     * @brief put a long64 in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putLong64(const key_t& key, const int64_t value);

    /**
     * @brief put a ulong64 in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putULong64(const key_t& key, const uint64_t value);

    /**
     * @brief put a float in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putFloat(const key_t& key, const float value);

    /**
     * @brief put a double in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putDouble(const key_t& key, const double value);

    /**
     * @brief put a bool in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putBool(const key_t& key, const bool value);

    /**
     * @brief put a C string in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putString(const key_t& key, const char * const value);

#ifdef ARDUINO
    /**
     * @brief put an Arduino string in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  value            value
     *
     * @returns the size of the inserted value
     */
    virtual size_t putString(const key_t& key, const String value);
#endif // ARDUINO

    /**
     * @brief get a char in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual int8_t      getChar(const key_t& key, const int8_t defaultValue = 0);

    /**
     * @brief get a uchar in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual uint8_t     getUChar(const key_t& key, const uint8_t defaultValue = 0);

    /**
     * @brief get a short in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual int16_t     getShort(const key_t& key, const int16_t defaultValue = 0);

    /**
     * @brief get a ushort in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual uint16_t    getUShort(const key_t& key, const uint16_t defaultValue = 0);

    /**
     * @brief get an int in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual int32_t     getInt(const key_t& key, const int32_t defaultValue = 0);

    /**
     * @brief get a uint in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual uint32_t    getUInt(const key_t& key, const uint32_t defaultValue = 0);

    /**
     * @brief get a long in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual int32_t     getLong(const key_t& key, const int32_t defaultValue = 0);

    /**
     * @brief get a ulong in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual uint32_t    getULong(const key_t& key, const uint32_t defaultValue = 0);

    /**
     * @brief get a long64 in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual int64_t     getLong64(const key_t& key, const int64_t defaultValue = 0);

    /**
     * @brief get a ulong64 in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual uint64_t    getULong64(const key_t& key, const uint64_t defaultValue = 0);

    /**
     * @brief get a float in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual float       getFloat(const key_t& key, const float defaultValue = NAN);

    /**
     * @brief get a double in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual double      getDouble(const key_t& key, const double defaultValue = NAN);

    /**
     * @brief get a bool in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual bool        getBool(const key_t& key, const bool defaultValue = false);

    /**
     * @brief get a C string in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual size_t      getString(const key_t& key, char* value, size_t maxLen);

#ifdef ARDUINO
    /**
     * @brief get an Arduino String in the kvstore
     *
     * @param[in]  key              Key
     * @param[in]  defaultValue     in the case the key do not exist this value is returned
     *
     * @returns the value present in the kvstore or defaultValue if not present
     */
    virtual String getString(const key_t& key, const String defaultValue = String());
#endif // ARDUINO

protected:
    // some implementations may need type-specific get and put methods, this can be performed by passing
    // type information as parameter to the get call and overcome the limitation of not being able to
    // override a templated method in cpp

    virtual res_t _put(const key_t& key, const uint8_t value[], size_t len, Type t);

    virtual res_t _get(const key_t& key, uint8_t value[], size_t len, Type t);
};

/* We need to ignore the warning "unused parameter" since we cannot add more instructions
 * like "(void) t;" because of some limitations cpp11 has with constexpr modifier
 * in this function we do not require to perform operations on the variable itself,
 * we need it just for the template to implicitly deduce the type of it and provide us with type information
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
template<typename T>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType(T t) {
    (void) t;
    return PT_INVALID;
}

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<int8_t>(int8_t t)                   { return PT_I8; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<uint8_t>(uint8_t t)                 { return PT_U8; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<bool>(bool t)                       { return PT_I8; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<int16_t>(int16_t t)                 { return PT_I16; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<uint16_t>(uint16_t t)               { return PT_U16; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<int32_t>(int32_t t)                 { return PT_I32; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<uint32_t>(uint32_t t)               { return PT_U32; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<int64_t>(int64_t t)                 { return PT_I64; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<uint64_t>(uint64_t t)               { return PT_U64; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<float>(float t)                     { return PT_FLOAT; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<double>(double t)                   { return PT_DOUBLE; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<char*>(char* t)                     { return PT_STR; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<char[]>(char t[])                   { return PT_STR; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<const char*>(const char* t)         { return PT_STR; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<uint8_t*>(uint8_t* t)               { return PT_BLOB; }

template<>
constexpr typename KVStoreInterface::Type KVStoreInterface::getType<const uint8_t*>(const uint8_t* t)   { return PT_BLOB; }

#pragma GCC diagnostic pop
