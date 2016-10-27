When sending messages from Arduinos, it must prepend all messages with a particular prefix so our USB detection driver can detect which Arduinos are for what purposes.

Messages should be five characters long and when making a new one, **must be added to this document**.

The list of things can be found below

| Prefix | Purpose |
| ------ | ------- |
| `GPS`  | GPS information |
