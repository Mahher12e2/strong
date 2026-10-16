let x: number;
x = 3;
const array: (number | string)[] = [1, "a", 2];
let a: boolean = true;
let b: string[] = ["1", "2"];
let c: any = 1;
c = "c";
c = false;
let d: number | string = 1;
const fun: (x: number, y: number) => number = (x: number, y: number) => x + y;
const fun2 = (s1: string, s2: string) => s1 + s2;
const fun3 = (s1: any, s2: any) => s1 + s2;
const fun4 = () => console.log(1);
type myType = number | string;

interface Person {
  name: string;
  age: number;
  sex: string;
  fun?: (x: number, y: number) => number;
}

let p: Person;
p = {
  name: "maher",
  age: 32,
  sex: "male",
  //fun:(x,y)=>x-y,
};
enum state {
  name = "maher",
  sex = "male",
}
let y: Person = {
  name: state.name,
  age: 33,
  sex: state.sex,
};
let statet = state.name;
interface Icar {
  name: string;
  model: number;
}
class Car implements Icar {
  name: string;
  model: number;
  constructor(x: string, n: number) {
    this.name = x;
    this.model = n;
  }
}

///
const sum: <t>(x: t, y: t) => t = <t>(x: t, y: t) => y;

sum<number>(1, 2);
sum<string>("1", "2");

let obj: { [key: string]: any } = {
  n: 2,
  y: 1,
};
obj.t = 0;
obj.gggggggggg = 0;

interface personal {
  name: string;
  email: string;
  number: number;
}

interface personalWithoutEmail extends Omit<personal, "email" | "number"> {}
let test: personalWithoutEmail = {
  name: "mm",
};
interface personalWithEmailOnly extends Pick<personal, "email"> {
  sex: string;
}
let test2: personalWithEmailOnly = {
  email: "mm",
  sex: "dd",
};
