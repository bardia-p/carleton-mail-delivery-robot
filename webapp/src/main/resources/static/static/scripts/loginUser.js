const submitButton = $("#loginUser-submit");
const errorMessage = $("#error-message");

/**
 * The JavaScript AJAX call for when a user is logged in.
 */
submitButton.click((e) => {
    e.preventDefault();
    const dataDictionary = {};
    dataDictionary["username"] = $("#username").val();
    dataDictionary["password"] = $("#password").val();
    const formData = JSON.stringify(dataDictionary);

    $.ajax({
        type: "POST",
        url: "/api/v1/loginUser",
        data: formData,
        dataType: "json",
        contentType: "application/json",
        success: function (res) {
            console.log('User registered successfully');
            if (res === 200) window.location.href = '/';
            else if (res === 401) errorMessage.text("Invalid Username or Password");
        },
        error: function (xhr, status, error) {
            // error handling
            console.error('Error logging in user:', error);
            setToast("error", "Something went wrong!", "Cannot log in at this time");
        }
    })

});